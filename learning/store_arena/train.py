"""매장 아레나 정책을 rsl_rl PPO 로 Newton 에서 학습한다 (headless, 청크 단위).

PPO 설정은 goal_reach 와 같고, 청크마다 --resume <model_*.pt> 로 이전 가중치와 옵티마이저를 이어 받아
--out 에 metrics.csv, model_*.pt, 텐서보드, run.json 을 쓴다 (run_train.sh 참고).
"""

import argparse
import json
import os
import sys
import time

import warp as wp

wp.config.enable_backward = False  # Isaac Lab train.py 와 같이 adjoint 커널 생략

from isaaclab.app import add_launcher_args, launch_simulation  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--num_envs", type=int, default=4096)
parser.add_argument("--max_iterations", type=int, default=300)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--out", type=str, required=True)
parser.add_argument("--resume", type=str, default="", help="model_*.pt of the previous chunk")
parser.add_argument(
    "--init",
    type=str,
    default="",
    help="start from these weights without the optimizer state (a widen_checkpoint.py output, "
    "or a plain checkpoint for the control run of the same recipe comparison)",
)
parser.add_argument(
    "--init_std",
    type=str,
    default="0",
    help="with --init: set the action std to this value, or to v,w values (exploration reset for "
    "a converged checkpoint whose std has shrunk); 0 keeps the std of the checkpoint",
)
parser.add_argument(
    "--gamma",
    type=float,
    default=0.0,
    help="PPO discount factor; 0 keeps agent_cfg (0.99, a horizon of 5 s at 20 Hz)",
)
parser.add_argument(
    "--entropy_coef", type=float, default=-1.0, help="PPO entropy bonus; negative keeps agent_cfg"
)
parser.add_argument(
    "--set", action="append", default=[], metavar="FIELD=VALUE", help="TaskSpec/ArenaSpec field"
)
add_launcher_args(parser)
args_cli = parser.parse_args()
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "goal_reach"))


def main():
    import arena_spec as arena
    import run_config
    import store_arena_env as sae

    init_std = run_config.parse_init_std(args_cli.init_std)
    if max(init_std) > 0 and (args_cli.resume or not args_cli.init):
        raise SystemExit("--init_std needs --init and no --resume (a resumed chunk keeps its std)")
    task_over, arena_over = arena.split_overrides(args_cli.set)
    env_cfg = sae.StoreArenaEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed
    env_cfg.spec_overrides = task_over
    env_cfg.arena_overrides = arena_over
    os.makedirs(args_cli.out, exist_ok=True)
    out = os.path.abspath(args_cli.out)
    env_cfg.metrics_csv = os.path.join(out, "metrics.csv")
    if os.path.exists(env_cfg.metrics_csv):
        os.remove(env_cfg.metrics_csv)

    with launch_simulation(env_cfg, args_cli):
        import torch
        from agent_cfg import GoalReachPPORunnerCfg
        from isaaclab_rl.rsl_rl import (
            RslRlVecEnvWrapper,
            check_rsl_rl_version,
            create_rsl_rl_runner,
            handle_deprecated_rsl_rl_cfg,
        )

        agent_cfg = GoalReachPPORunnerCfg()
        agent_cfg.experiment_name = "pode_store_arena"
        agent_cfg.max_iterations = args_cli.max_iterations
        agent_cfg.seed = args_cli.seed
        if args_cli.gamma > 0:
            agent_cfg.algorithm.gamma = args_cli.gamma
        if args_cli.entropy_coef >= 0:
            agent_cfg.algorithm.entropy_coef = args_cli.entropy_coef
        if arena.ts.make_spec(task_over).critic_privileged:  # env 가 "critic" 그룹을 더함
            agent_cfg.obs_groups = {"actor": ["policy"], "critic": ["policy", "critic"]}
        agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, check_rsl_rl_version())
        env_cfg.metrics_every = agent_cfg.num_steps_per_env
        # 설정 스냅샷: 학습 전에 써서 중간에 멈춘 실행도 무엇으로 돌았는지 남는다
        config = run_config.build(
            argv=sys.argv,
            sets=args_cli.set,
            num_envs=args_cli.num_envs,
            max_iterations=args_cli.max_iterations,
            seed=args_cli.seed,
            init=args_cli.init,
            resume=args_cli.resume,
            init_std=args_cli.init_std,
            task_spec=arena.ts.make_spec(task_over),
            arena_spec=arena.make_arena_spec(arena_over),
            agent=agent_cfg.to_dict() if hasattr(agent_cfg, "to_dict") else dict(agent_cfg),
        )
        config_sha = run_config.write(os.path.join(out, "run_config.json"), config)

        t0 = time.time()
        env = sae.StoreArenaEnv(env_cfg)
        t_env = time.time() - t0
        routes = env.routes
        env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
        runner = create_rsl_rl_runner(env, agent_cfg, log_dir=out)
        if args_cli.resume:
            runner.load(args_cli.resume)
        elif args_cli.init:
            no_opt = {"actor": True, "critic": True, "optimizer": False, "iteration": True}
            runner.load(args_cli.init, load_cfg=no_opt)
            if max(init_std) > 0:
                # 탐색 되살리기: 수렴한 체크포인트의 줄어든 행동 std 를 다시 키운다
                dist = runner.alg.get_policy().distribution
                old_std = dist.std_param.detach().cpu().tolist()
                with torch.no_grad():
                    for k, value in enumerate(init_std):
                        if value > 0:
                            dist.std_param[k] = value
                print(
                    f"[ARENA] action std reset {old_std} -> {dist.std_param.detach().cpu().tolist()}"
                )
        t1 = time.time()
        runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)
        torch.cuda.synchronize()
        t_learn = time.time() - t1
        steps = agent_cfg.max_iterations * agent_cfg.num_steps_per_env * args_cli.num_envs
        info = {
            "num_envs": args_cli.num_envs,
            "max_iterations": agent_cfg.max_iterations,
            "num_steps_per_env": agent_cfg.num_steps_per_env,
            "seed": args_cli.seed,
            "resume": args_cli.resume,
            "init": args_cli.init,
            "init_std": list(init_std),
            "run_config_sha256": config_sha,
            "spec_overrides": task_over,
            "arena_overrides": arena_over,
            "routes": routes.count,
            "env_build_s": round(t_env, 1),
            "learn_s": round(t_learn, 1),
            "env_steps": steps,
            "env_steps_per_s_incl_update": round(steps / t_learn, 0),
            "cuda_max_mem_alloc_MB": round(torch.cuda.max_memory_allocated() / 2**20, 1),
        }
        with open(os.path.join(out, "run.json"), "w", encoding="utf-8") as f:
            json.dump(info, f, indent=1)
        print("[ARENA] run info:", json.dumps(info))
        env.close()


if __name__ == "__main__":
    main()
