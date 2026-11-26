# n26 표

실행 폴더 `D:/isaac/runs/store_arena/n26`, 끝난 실행(exit=0): c_clean, c_cross, c_demo_box, e_acd3_aisle, e_acd3_aislet, e_acd3_demo1, e_acd3_demo1c08, e_acd3_demo1t, e_acd3_demo2, e_acd3_head, e_acd3_rand, e_acd3_rand_chk2, e_ctrl_aisle, e_ctrl_demo1, e_ctrl_demo2, e_ctrl_rand, e_demo_aisle, e_demo_demo1, e_demo_demo2, e_demo_head, e_demo_rand, e_demohist2_aisle, e_demohist2_demo1, e_demohist2_demo2, e_demohist2_head, e_demohist2_rand, e_demohist2m_demo1, e_demohist_aisle, e_demohist_aislel958, e_demohist_aislepost, e_demohist_aislepostm, e_demohist_aislepostmf, e_demohist_aislepostn, e_demohist_aislet, e_demohist_demo1, e_demohist_demo1c08, e_demohist_demo1l958, e_demohist_demo1maskw, e_demohist_demo1post, e_demohist_demo1postm, e_demohist_demo1postmf, e_demohist_demo1postn, e_demohist_demo1t, e_demohist_demo1tc26a60, e_demohist_demo1tc26a90, e_demohist_demo1tf15a60, e_demohist_demo1tf20a30, e_demohist_demo1tf20a60, e_demohist_demo1tp30a60, e_demohist_demo1tp30a90, e_demohist_demo1tw26a60, e_demohist_demo1tw26a60n, e_demohist_demo2, e_demohist_head, e_demohist_rand, e_demohistdyn_aisle, e_demohistdyn_demo1, e_demohistdyn_demo2, e_demohistdyn_rand, e_demostd2_aisle, e_demostd2_demo1, e_demostd2_demo2, e_demostd2_rand, e_demostd2m_demo1, e_demostd_aisle, e_demostd_demo1, e_demostd_demo2, e_demostd_head, e_demostd_rand, e_histstd_aisle, e_histstd_demo1, e_histstd_demo2, e_histstd_head, e_histstd_rand, e_std_aisle, e_std_aislet, e_std_demo1, e_std_demo1c08, e_std_demo1t, e_std_demo2, e_std_head, e_std_rand, smoke_b11, smoke_b4, smoke_demo, smoke_hist, smoke_hstd, smoke_resume, smoke_std, smoke_sysid, t_ctrl, t_demo, t_demo_hist, t_demo_hist2, t_demo_hist_dyn, t_demo_std, t_demo_std2, t_dyn, t_hist_std, t_std

## 1. 평가 칸


| 정책 | 칸 | 에피소드 | 성공 % | 충돌 % | 시간 초과 % | 충돌 벽·선반 | 충돌 상자 | 충돌 사람·카트 | 충돌 로봇 | 스침만 | w 반전 회/분 | 성공 시간/최고속 경로 시간 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| acd3 | aisle | 4096 | 56.62 | 35.62 | 7.76 | 221 | 1238 | 0 | 0 | 1375 | 55.65 | 1.476 |
| acd3 | aislet | 4096 | 56.64 | 35.52 | 7.84 | 219 | 1236 | 0 | 0 | 1368 | 55.9 | 1.476 |
| acd3 | demo1 | 4096 | 43.65 | 52.73 | 3.61 | 284 | 907 | 436 | 533 | 2088 | 51.5 | 1.558 |
| acd3 | demo1c08 | 4096 | 44.63 | 50.05 | 5.32 | 253 | 849 | 490 | 458 | 2020 | 71.71 | 1.386 |
| acd3 | demo1t | 4096 | 43.8 | 52.56 | 3.64 | 281 | 909 | 447 | 516 | 2084 | 51.42 | 1.561 |
| acd3 | demo2 | 4096 | 40.31 | 53.61 | 6.08 | 370 | 895 | 403 | 528 | 2131 | 56.2 | 1.522 |
| acd3 | head | 4096 | 47.41 | 52.17 | 0.42 | 195 | 40 | 414 | 1488 | 1970 | 45.15 | 1.407 |
| acd3 | rand | 4096 | 76.32 | 23.27 | 0.42 | 114 | 45 | 436 | 358 | 896 | 43.3 | 1.158 |
| acd3 | rand_chk2 | 4096 | 76.29 | 23.34 | 0.37 | 114 | 46 | 442 | 354 | 902 | 43.21 | 1.159 |
| ctrl | aisle | 4096 | 55.35 | 35.28 | 9.38 | 214 | 1231 | 0 | 0 | 1386 | 60.49 | 1.478 |
| ctrl | demo1 | 4096 | 43.33 | 53.39 | 3.27 | 258 | 1035 | 390 | 504 | 2103 | 51.23 | 1.512 |
| ctrl | demo2 | 4096 | 39.7 | 55.74 | 4.57 | 369 | 1006 | 361 | 547 | 2190 | 53.81 | 1.481 |
| ctrl | rand | 4096 | 75.95 | 23.29 | 0.76 | 86 | 50 | 453 | 365 | 897 | 39.81 | 1.142 |
| demo | aisle | 4096 | 55.98 | 39.89 | 4.13 | 211 | 1423 | 0 | 0 | 1485 | 48.26 | 1.436 |
| demo | demo1 | 4096 | 41.65 | 57.5 | 0.85 | 253 | 1162 | 393 | 547 | 2215 | 42.77 | 1.439 |
| demo | demo2 | 4096 | 39.36 | 58.81 | 1.83 | 393 | 1096 | 369 | 551 | 2265 | 45.12 | 1.43 |
| demo | head | 4096 | 45.07 | 54.39 | 0.54 | 185 | 43 | 431 | 1569 | 1905 | 38.52 | 1.333 |
| demo | rand | 4096 | 75.02 | 24.58 | 0.39 | 99 | 51 | 473 | 384 | 890 | 39.13 | 1.146 |
| demohist | aisle | 4096 | 72.19 | 25.02 | 2.78 | 171 | 854 | 0 | 0 | 844 | 44.3 | 1.213 |
| demohist | aislel958 | 4096 | 73.27 | 23.8 | 2.93 | 180 | 795 | 0 | 0 | 812 | 44.49 | 1.225 |
| demohist | aislepost | 4096 | 67.68 | 28.88 | 3.44 | 279 | 904 | 0 | 0 | 1068 | 47.77 | 1.366 |
| demohist | aislepostm | 4096 | 33.94 | 63.7 | 2.37 | 1900 | 709 | 0 | 0 | 2342 | 38.64 | 1.395 |
| demohist | aislepostmf | 4096 | 71.8 | 25.34 | 2.86 | 154 | 884 | 0 | 0 | 889 | 44.17 | 1.209 |
| demohist | aislepostn | 4096 | 71.83 | 25.2 | 2.98 | 173 | 859 | 0 | 0 | 859 | 44.27 | 1.224 |
| demohist | aislet | 4096 | 72.22 | 24.9 | 2.88 | 168 | 852 | 0 | 0 | 835 | 44.35 | 1.213 |
| demohist | demo1 | 4096 | 52.81 | 46.02 | 1.17 | 232 | 774 | 324 | 555 | 1729 | 40.22 | 1.309 |
| demohist | demo1c08 | 4096 | 52.05 | 46.9 | 1.05 | 222 | 728 | 457 | 514 | 1825 | 50.25 | 1.159 |
| demohist | demo1l958 | 4096 | 52.95 | 46.04 | 1.0 | 234 | 760 | 336 | 556 | 1734 | 40.11 | 1.309 |
| demohist | demo1maskw | 4096 | 15.82 | 83.52 | 0.66 | 2106 | 482 | 240 | 593 | 3075 | 29.49 | 1.371 |
| demohist | demo1post | 4096 | 48.75 | 50.34 | 0.9 | 333 | 853 | 361 | 515 | 1898 | 41.96 | 1.428 |
| demohist | demo1postm | 4096 | 16.72 | 82.52 | 0.76 | 2063 | 493 | 232 | 592 | 3001 | 28.99 | 1.353 |
| demohist | demo1postmf | 4096 | 51.59 | 47.22 | 1.2 | 215 | 817 | 354 | 548 | 1745 | 40.78 | 1.288 |
| demohist | demo1postn | 4096 | 52.2 | 46.83 | 0.98 | 232 | 793 | 338 | 555 | 1742 | 40.96 | 1.303 |
| demohist | demo1t | 4096 | 52.54 | 46.39 | 1.07 | 232 | 777 | 337 | 554 | 1739 | 40.43 | 1.305 |
| demohist | demo1tc26a60 | 4096 | 53.2 | 45.58 | 1.22 | 242 | 814 | 281 | 530 | 1725 | 41.19 | 1.351 |
| demohist | demo1tc26a90 | 4096 | 51.98 | 46.88 | 1.15 | 245 | 798 | 362 | 515 | 1781 | 41.82 | 1.385 |
| demohist | demo1tf15a60 | 4096 | 28.27 | 24.24 | 47.49 | 119 | 61 | 482 | 331 | 986 | 91.38 | 1.459 |
| demohist | demo1tf20a30 | 4096 | 35.74 | 24.8 | 39.45 | 140 | 29 | 507 | 340 | 1002 | 81.78 | 1.7 |
| demohist | demo1tf20a60 | 4096 | 17.8 | 18.73 | 63.48 | 89 | 21 | 263 | 394 | 756 | 82.3 | 1.964 |
| demohist | demo1tp30a60 | 4096 | 56.4 | 42.31 | 1.29 | 231 | 817 | 106 | 579 | 1580 | 45.09 | 1.367 |
| demohist | demo1tp30a90 | 4096 | 55.93 | 42.9 | 1.17 | 222 | 829 | 134 | 572 | 1598 | 45.63 | 1.376 |
| demohist | demo1tw26a60 | 4096 | 57.84 | 40.82 | 1.34 | 278 | 800 | 86 | 508 | 1513 | 45.62 | 1.441 |
| demohist | demo1tw26a60n | 4096 | 57.62 | 41.19 | 1.2 | 271 | 807 | 89 | 520 | 1524 | 46.83 | 1.452 |
| demohist | demo2 | 4096 | 48.27 | 50.44 | 1.29 | 368 | 823 | 340 | 535 | 1854 | 39.82 | 1.306 |
| demohist | head | 4096 | 51.12 | 48.88 | 0.0 | 230 | 24 | 432 | 1316 | 1749 | 46.69 | 1.282 |
| demohist | rand | 4096 | 77.37 | 22.56 | 0.07 | 60 | 26 | 457 | 381 | 829 | 51.56 | 1.117 |
| demohist2 | aisle | 4096 | 73.95 | 23.56 | 2.49 | 135 | 830 | 0 | 0 | 876 | 45.22 | 1.2 |
| demohist2 | demo1 | 4096 | 54.27 | 44.34 | 1.39 | 227 | 737 | 361 | 491 | 1711 | 39.66 | 1.275 |
| demohist2 | demo2 | 4096 | 50.05 | 48.44 | 1.51 | 351 | 821 | 326 | 486 | 1857 | 38.24 | 1.287 |
| demohist2 | head | 4096 | 49.27 | 50.71 | 0.02 | 156 | 29 | 403 | 1489 | 1891 | 44.0 | 1.242 |
| demohist2 | rand | 4096 | 78.54 | 21.44 | 0.02 | 68 | 25 | 410 | 375 | 826 | 48.22 | 1.124 |
| demohist2m | demo1 | 4096 | 54.76 | 44.29 | 0.95 | 221 | 746 | 348 | 499 | 1673 | 40.39 | 1.278 |
| demohistdyn | aisle | 4096 | 68.16 | 30.32 | 1.51 | 119 | 1123 | 0 | 0 | 1072 | 41.47 | 1.223 |
| demohistdyn | demo1 | 4096 | 49.05 | 50.54 | 0.42 | 212 | 931 | 389 | 538 | 1883 | 38.78 | 1.272 |
| demohistdyn | demo2 | 4096 | 45.58 | 54.17 | 0.24 | 354 | 982 | 355 | 528 | 2007 | 37.75 | 1.279 |
| demohistdyn | rand | 4096 | 76.15 | 23.78 | 0.07 | 88 | 45 | 486 | 355 | 839 | 49.77 | 1.109 |
| demostd | aisle | 4096 | 70.41 | 28.17 | 1.42 | 100 | 1054 | 0 | 0 | 977 | 43.59 | 1.223 |
| demostd | demo1 | 4096 | 51.25 | 48.29 | 0.46 | 191 | 903 | 368 | 516 | 1811 | 38.48 | 1.289 |
| demostd | demo2 | 4096 | 47.8 | 51.25 | 0.95 | 328 | 914 | 359 | 498 | 1916 | 36.88 | 1.29 |
| demostd | head | 4096 | 48.1 | 51.86 | 0.05 | 201 | 26 | 431 | 1466 | 1846 | 46.45 | 1.287 |
| demostd | rand | 4096 | 77.51 | 22.41 | 0.07 | 68 | 27 | 447 | 376 | 817 | 50.68 | 1.121 |
| demostd2 | aisle | 4096 | 72.92 | 24.56 | 2.51 | 75 | 931 | 0 | 0 | 894 | 41.3 | 1.221 |
| demostd2 | demo1 | 4096 | 53.3 | 45.43 | 1.27 | 192 | 803 | 377 | 489 | 1703 | 38.43 | 1.29 |
| demostd2 | demo2 | 4096 | 49.27 | 49.0 | 1.73 | 296 | 863 | 365 | 483 | 1870 | 38.29 | 1.296 |
| demostd2 | rand | 4096 | 78.61 | 21.34 | 0.05 | 55 | 28 | 437 | 354 | 802 | 49.27 | 1.121 |
| demostd2m | demo1 | 4096 | 51.73 | 47.49 | 0.78 | 202 | 885 | 356 | 502 | 1788 | 38.04 | 1.276 |
| histstd | aisle | 4096 | 68.65 | 27.34 | 4.0 | 161 | 959 | 0 | 0 | 1004 | 48.32 | 1.353 |
| histstd | demo1 | 4096 | 51.27 | 47.44 | 1.29 | 238 | 826 | 407 | 472 | 1788 | 40.64 | 1.428 |
| histstd | demo2 | 4096 | 47.12 | 51.1 | 1.78 | 362 | 883 | 373 | 475 | 1930 | 41.61 | 1.421 |
| histstd | head | 4096 | 49.27 | 50.73 | 0.0 | 209 | 37 | 393 | 1439 | 1843 | 46.4 | 1.376 |
| histstd | rand | 4096 | 77.98 | 21.92 | 0.1 | 54 | 36 | 433 | 375 | 810 | 48.66 | 1.142 |
| std | aisle | 4096 | 67.9 | 22.66 | 9.45 | 171 | 757 | 0 | 0 | 842 | 66.75 | 1.368 |
| std | aislet | 4096 | 67.94 | 22.56 | 9.5 | 172 | 752 | 0 | 0 | 837 | 67.76 | 1.371 |
| std | demo1 | 4096 | 52.49 | 43.8 | 3.71 | 254 | 703 | 379 | 458 | 1698 | 47.82 | 1.433 |
| std | demo1c08 | 4096 | 53.86 | 43.26 | 2.88 | 260 | 613 | 449 | 450 | 1696 | 59.01 | 1.276 |
| std | demo1t | 4096 | 52.61 | 43.7 | 3.69 | 254 | 712 | 376 | 448 | 1701 | 47.44 | 1.441 |
| std | demo2 | 4096 | 48.49 | 46.92 | 4.59 | 372 | 724 | 342 | 484 | 1825 | 48.36 | 1.472 |
| std | head | 4096 | 52.54 | 47.09 | 0.37 | 181 | 32 | 371 | 1345 | 1772 | 46.64 | 1.433 |
| std | rand | 4096 | 79.27 | 20.48 | 0.24 | 55 | 25 | 376 | 383 | 778 | 47.22 | 1.146 |

## 2. 짝 비교 (a - 대조)

에피소드별 짝 차이와 SE. 2SE가 outside면 |차이| > 2 SE. 같은 에피소드 확인은 시작 상태가 저장된 추적 env(괄호 안 수), env 번호, 경로 표(풀 전체의 전역 경로와 상자 자리)를 본다. 종류별 충돌 차는 (충돌했고 그 순간 로봇 중심에 가장 가까운 것이 그 종류) 여부의 에피소드별 짝 차이이며, 두 실행 모두 episodes.npz에 hit_kind가 있을 때만 적고 없는 실행이 낀 줄은 빈칸이다.

| 정책 | 칸 | 대조 정책 | 대조 칸 | 짝 수 | 같은 에피소드 | 성공 차 %p | 성공 SE | 성공 2SE | 충돌 차 %p | 충돌 SE | 충돌 2SE | 시간 초과 차 %p | 시간 초과 SE | 시간 초과 2SE | 벽·선반 충돌 차 %p | 벽·선반 충돌 SE | 벽·선반 충돌 2SE | 상자 충돌 차 %p | 상자 충돌 SE | 상자 충돌 2SE | 사람·카트 충돌 차 %p | 사람·카트 충돌 SE | 사람·카트 충돌 2SE | 로봇 충돌 차 %p | 로봇 충돌 SE | 로봇 충돌 2SE |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| std | rand | ctrl | rand | 4096 | yes (16 tracked envs) | 3.3203 | 0.4642 | outside | -2.8076 | 0.4593 | outside | -0.5127 | 0.1442 | outside |  |  |  |  |  |  |  |  |  |  |  |  |
| ctrl | rand | acd3 | rand | 4096 | yes (16 tracked envs) | -0.3662 | 0.3427 | inside | 0.0244 | 0.3679 | inside | 0.3418 | 0.1543 | outside |  |  |  |  |  |  |  |  |  |  |  |  |
| std | rand | acd3 | rand | 4096 | yes (16 tracked envs) | 2.9541 | 0.468 | outside | -2.7832 | 0.4702 | outside | -0.1709 | 0.1119 | inside |  |  |  |  |  |  |  |  |  |  |  |  |
| std | head | acd3 | head | 4096 | yes (16 tracked envs) | 5.127 | 0.6105 | outside | -5.0781 | 0.6097 | outside | -0.0488 | 0.1381 | inside |  |  |  |  |  |  |  |  |  |  |  |  |
| demo | rand | ctrl | rand | 4096 | yes (16 tracked envs) | -0.9277 | 0.3502 | outside | 1.2939 | 0.3507 | outside | -0.3662 | 0.1314 | outside |  |  |  |  |  |  |  |  |  |  |  |  |
| demostd | rand | std | rand | 4096 | yes (16 tracked envs) | -1.7578 | 0.4373 | outside | 1.9287 | 0.4405 | outside | -0.1709 | 0.0809 | outside |  |  |  |  |  |  |  |  |  |  |  |  |
| demostd | rand | demo | rand | 4096 | yes (16 tracked envs) | 2.4902 | 0.4551 | outside | -2.1729 | 0.4536 | outside | -0.3174 | 0.1063 | outside | -0.7568 | 0.2638 | outside | -0.5859 | 0.1542 | outside | -0.6348 | 0.3734 | inside | -0.1953 | 0.2584 | inside |
| demo | rand | acd3 | rand | 4096 | yes (16 tracked envs) | -1.2939 | 0.3722 | outside | 1.3184 | 0.384 | outside | -0.0244 | 0.1269 | inside |  |  |  |  |  |  |  |  |  |  |  |  |
| demostd | rand | acd3 | rand | 4096 | yes (16 tracked envs) | 1.1963 | 0.4762 | outside | -0.8545 | 0.4764 | inside | -0.3418 | 0.1091 | outside |  |  |  |  |  |  |  |  |  |  |  |  |
| demostd | head | std | head | 4096 | yes (16 tracked envs) | -4.4434 | 0.6079 | outside | 4.7607 | 0.6127 | outside | -0.3174 | 0.1006 | outside |  |  |  |  |  |  |  |  |  |  |  |  |
| demostd | head | demo | head | 4096 | yes (16 tracked envs) | 3.0273 | 0.5601 | outside | -2.5391 | 0.5586 | outside | -0.4883 | 0.1194 | outside | 0.3906 | 0.3588 | inside | -0.415 | 0.1401 | outside | 0.0 | 0.3538 | inside | -2.5146 | 0.5147 | outside |
| demo | head | acd3 | head | 4096 | yes (16 tracked envs) | -2.3438 | 0.4566 | outside | 2.2217 | 0.46 | outside | 0.1221 | 0.1444 | inside |  |  |  |  |  |  |  |  |  |  |  |  |
| demostd | head | acd3 | head | 4096 | yes (16 tracked envs) | 0.6836 | 0.592 | inside | -0.3174 | 0.5895 | inside | -0.3662 | 0.1063 | outside |  |  |  |  |  |  |  |  |  |  |  |  |
| demo | aisle | ctrl | aisle | 4096 | yes (16 tracked envs) | 0.6348 | 0.4554 | inside | 4.6143 | 0.5644 | outside | -5.249 | 0.493 | outside | -0.0732 | 0.3117 | inside | 4.6875 | 0.5605 | outside | 0.0 | 0.0 | inside | 0.0 | 0.0 | inside |
| demo | aisle | acd3 | aisle | 4096 | yes (16 tracked envs) | -0.6348 | 0.4085 | inside | 4.2725 | 0.5165 | outside | -3.6377 | 0.4365 | outside | -0.2441 | 0.303 | inside | 4.5166 | 0.5342 | outside | 0.0 | 0.0 | inside | 0.0 | 0.0 | inside |
| demostd | aisle | std | aisle | 4096 | yes (16 tracked envs) | 2.5146 | 0.5418 | outside | 5.5176 | 0.6146 | outside | -8.0322 | 0.4559 | outside | -1.7334 | 0.3468 | outside | 7.251 | 0.6383 | outside | 0.0 | 0.0 | inside | 0.0 | 0.0 | inside |
| demostd | aisle | demo | aisle | 4096 | yes (16 tracked envs) | 14.4287 | 0.6632 | outside | -11.7188 | 0.6827 | outside | -2.71 | 0.333 | outside | -2.71 | 0.3555 | outside | -9.0088 | 0.725 | outside | 0.0 | 0.0 | inside | 0.0 | 0.0 | inside |
| demostd | aisle | acd3 | aisle | 4096 | yes (16 tracked envs) | 13.7939 | 0.6727 | outside | -7.4463 | 0.712 | outside | -6.3477 | 0.4268 | outside | -2.9541 | 0.3682 | outside | -4.4922 | 0.7525 | outside | 0.0 | 0.0 | inside | 0.0 | 0.0 | inside |
| ctrl | aisle | acd3 | aisle | 4096 | yes (16 tracked envs) | -1.2695 | 0.4471 | outside | -0.3418 | 0.5449 | inside | 1.6113 | 0.4828 | outside | -0.1709 | 0.3117 | inside | -0.1709 | 0.5277 | inside | 0.0 | 0.0 | inside | 0.0 | 0.0 | inside |
| std | aisle | acd3 | aisle | 4096 | yes (16 tracked envs) | 11.2793 | 0.6579 | outside | -12.9639 | 0.7463 | outside | 1.6846 | 0.5663 | outside | -1.2207 | 0.3978 | outside | -11.7432 | 0.763 | outside | 0.0 | 0.0 | inside | 0.0 | 0.0 | inside |
| std | aisle | ctrl | aisle | 4096 | yes (16 tracked envs) | 12.5488 | 0.6577 | outside | -12.6221 | 0.7389 | outside | 0.0732 | 0.5616 | inside | -1.0498 | 0.3926 | outside | -11.5723 | 0.7499 | outside | 0.0 | 0.0 | inside | 0.0 | 0.0 | inside |
| demo | demo1 | ctrl | demo1 | 4096 | yes (16 tracked envs) | -1.6846 | 0.4734 | outside | 4.1016 | 0.5356 | outside | -2.417 | 0.2936 | outside | -0.1221 | 0.379 | inside | 3.1006 | 0.5367 | outside | 0.0732 | 0.3822 | inside | 1.0498 | 0.4344 | outside |
| demo | demo1 | acd3 | demo1 | 4096 | yes (16 tracked envs) | -2.002 | 0.4505 | outside | 4.7607 | 0.5247 | outside | -2.7588 | 0.3068 | outside | -0.7568 | 0.3867 | inside | 6.2256 | 0.5389 | outside | -1.0498 | 0.3911 | outside | 0.3418 | 0.4313 | inside |
| demostd | demo1 | std | demo1 | 4096 | yes (16 tracked envs) | -1.2451 | 0.5933 | outside | 4.4922 | 0.6477 | outside | -3.2471 | 0.3037 | outside | -1.5381 | 0.4368 | outside | 4.8828 | 0.6159 | outside | -0.2686 | 0.399 | inside | 1.416 | 0.4878 | outside |
| demostd | demo1 | demo | demo1 | 4096 | yes (16 tracked envs) | 9.5947 | 0.6336 | outside | -9.2041 | 0.648 | outside | -0.3906 | 0.1655 | outside | -1.5137 | 0.4416 | outside | -6.3232 | 0.6917 | outside | -0.6104 | 0.4092 | inside | -0.7568 | 0.4949 | inside |
| demostd | demo1 | acd3 | demo1 | 4096 | yes (16 tracked envs) | 7.5928 | 0.663 | outside | -4.4434 | 0.7093 | outside | -3.1494 | 0.3 | outside | -2.2705 | 0.4574 | outside | -0.0977 | 0.7119 | inside | -1.6602 | 0.4638 | outside | -0.415 | 0.5254 | inside |
| ctrl | demo1 | acd3 | demo1 | 4096 | yes (16 tracked envs) | -0.3174 | 0.4374 | inside | 0.6592 | 0.5138 | inside | -0.3418 | 0.3275 | inside | -0.6348 | 0.3571 | inside | 3.125 | 0.4847 | outside | -1.123 | 0.3361 | outside | -0.708 | 0.3928 | inside |
| std | demo1 | acd3 | demo1 | 4096 | yes (16 tracked envs) | 8.8379 | 0.6651 | outside | -8.9355 | 0.7387 | outside | 0.0977 | 0.383 | inside | -0.7324 | 0.4619 | inside | -4.9805 | 0.6845 | outside | -1.3916 | 0.4464 | outside | -1.8311 | 0.5336 | outside |
| std | demo1 | ctrl | demo1 | 4096 | yes (16 tracked envs) | 9.1553 | 0.66 | outside | -9.5947 | 0.724 | outside | 0.4395 | 0.3571 | inside | -0.0977 | 0.4529 | inside | -8.1055 | 0.6798 | outside | -0.2686 | 0.4107 | inside | -1.123 | 0.5234 | outside |
| demo | demo2 | ctrl | demo2 | 4096 | yes (16 tracked envs) | -0.3418 | 0.4143 | inside | 3.0762 | 0.5134 | outside | -2.7344 | 0.3356 | outside | 0.5859 | 0.4448 | inside | 2.1973 | 0.5405 | outside | 0.1953 | 0.3703 | inside | 0.0977 | 0.434 | inside |
| demo | demo2 | acd3 | demo2 | 4096 | yes (16 tracked envs) | -0.9521 | 0.409 | outside | 5.2002 | 0.5259 | outside | -4.248 | 0.3676 | outside | 0.5615 | 0.4374 | inside | 4.9072 | 0.5509 | outside | -0.8301 | 0.3827 | outside | 0.5615 | 0.4401 | inside |
| demostd | demo2 | std | demo2 | 4096 | yes (16 tracked envs) | -0.6836 | 0.5588 | inside | 4.3213 | 0.627 | outside | -3.6377 | 0.3415 | outside | -1.0742 | 0.506 | outside | 4.6387 | 0.6278 | outside | 0.415 | 0.3868 | inside | 0.3418 | 0.5063 | inside |
| demostd | demo2 | demo | demo2 | 4096 | yes (16 tracked envs) | 8.4473 | 0.5864 | outside | -7.5684 | 0.6237 | outside | -0.8789 | 0.2462 | outside | -1.5869 | 0.4883 | outside | -4.4434 | 0.6678 | outside | -0.2441 | 0.4012 | inside | -1.2939 | 0.5181 | outside |
| demostd | demo2 | acd3 | demo2 | 4096 | yes (16 tracked envs) | 7.4951 | 0.5941 | outside | -2.3682 | 0.6857 | outside | -5.127 | 0.3839 | outside | -1.0254 | 0.5166 | inside | 0.4639 | 0.6996 | inside | -1.0742 | 0.4365 | outside | -0.7324 | 0.5326 | inside |
| ctrl | demo2 | acd3 | demo2 | 4096 | yes (16 tracked envs) | -0.6104 | 0.3726 | inside | 2.124 | 0.5023 | outside | -1.5137 | 0.3869 | outside | -0.0244 | 0.4222 | inside | 2.71 | 0.4932 | outside | -1.0254 | 0.3235 | outside | 0.4639 | 0.3989 | inside |
| std | demo2 | acd3 | demo2 | 4096 | yes (16 tracked envs) | 8.1787 | 0.6058 | outside | -6.6895 | 0.7184 | outside | -1.4893 | 0.4503 | outside | 0.0488 | 0.5416 | inside | -4.1748 | 0.694 | outside | -1.4893 | 0.4101 | outside | -1.0742 | 0.538 | inside |
| std | demo2 | ctrl | demo2 | 4096 | yes (16 tracked envs) | 8.7891 | 0.6022 | outside | -8.8135 | 0.6954 | outside | 0.0244 | 0.4108 | inside | 0.0732 | 0.5231 | inside | -6.8848 | 0.6778 | outside | -0.4639 | 0.3883 | inside | -1.5381 | 0.5327 | outside |
| demohist | demo1 | demostd | demo1 | 4096 | yes (16 tracked envs) | 1.5625 | 0.5552 | outside | -2.2705 | 0.5783 | outside | 0.708 | 0.1706 | outside | 1.001 | 0.4061 | outside | -3.1494 | 0.5679 | outside | -1.0742 | 0.3534 | outside | 0.9521 | 0.4726 | outside |
| demohist | demo1 | acd3 | demo1 | 4096 | yes (16 tracked envs) | 9.1553 | 0.684 | outside | -6.7139 | 0.7368 | outside | -2.4414 | 0.3217 | outside | -1.2695 | 0.4564 | outside | -3.2471 | 0.6969 | outside | -2.7344 | 0.4574 | outside | 0.5371 | 0.5259 | inside |
| demohist | demo1 | std | demo1 | 4096 | yes (16 tracked envs) | 0.3174 | 0.6006 | inside | 2.2217 | 0.6638 | outside | -2.5391 | 0.3233 | outside | -0.5371 | 0.4421 | inside | 1.7334 | 0.6205 | outside | -1.3428 | 0.3878 | outside | 2.3682 | 0.4996 | outside |
| demohist | demo2 | demostd | demo2 | 4096 | yes (16 tracked envs) | 0.4639 | 0.5057 | inside | -0.8057 | 0.5442 | inside | 0.3418 | 0.2183 | inside | 0.9766 | 0.4669 | outside | -2.2217 | 0.5773 | outside | -0.4639 | 0.3512 | inside | 0.9033 | 0.4764 | inside |
| demohist | demo2 | acd3 | demo2 | 4096 | yes (16 tracked envs) | 7.959 | 0.6273 | outside | -3.1738 | 0.7193 | outside | -4.7852 | 0.3972 | outside | -0.0488 | 0.5237 | inside | -1.7578 | 0.713 | outside | -1.5381 | 0.4502 | outside | 0.1709 | 0.5552 | inside |
| demohist | demo2 | std | demo2 | 4096 | yes (16 tracked envs) | -0.2197 | 0.5637 | inside | 3.5156 | 0.6427 | outside | -3.2959 | 0.3475 | outside | -0.0977 | 0.518 | inside | 2.417 | 0.6342 | outside | -0.0488 | 0.3798 | inside | 1.2451 | 0.5307 | outside |
| demohist | aisle | demostd | aisle | 4096 | yes (16 tracked envs) | 1.7822 | 0.4808 | outside | -3.1494 | 0.5288 | outside | 1.3672 | 0.2552 | outside | 1.7334 | 0.3434 | outside | -4.8828 | 0.5902 | outside | 0.0 | 0.0 | inside | 0.0 | 0.0 | inside |
| demohist | aisle | acd3 | aisle | 4096 | yes (16 tracked envs) | 15.5762 | 0.7121 | outside | -10.5957 | 0.7597 | outside | -4.9805 | 0.4541 | outside | -1.2207 | 0.384 | outside | -9.375 | 0.7867 | outside | 0.0 | 0.0 | inside | 0.0 | 0.0 | inside |
| demohist | aisle | std | aisle | 4096 | yes (16 tracked envs) | 4.2969 | 0.5676 | outside | 2.3682 | 0.619 | outside | -6.665 | 0.4468 | outside | 0.0 | 0.3572 | inside | 2.3682 | 0.6398 | outside | 0.0 | 0.0 | inside | 0.0 | 0.0 | inside |
| demohist | rand | demostd | rand | 4096 | yes (16 tracked envs) | -0.1465 | 0.4201 | inside | 0.1465 | 0.4215 | inside | 0.0 | 0.0488 | inside | -0.1953 | 0.2342 | inside | -0.0244 | 0.1171 | inside | 0.2441 | 0.3487 | inside | 0.1221 | 0.2549 | inside |
| demohist | rand | acd3 | rand | 4096 | yes (16 tracked envs) | 1.0498 | 0.4713 | outside | -0.708 | 0.4702 | inside | -0.3418 | 0.1035 | outside |  |  |  |  |  |  |  |  |  |  |  |  |
| demohist | rand | std | rand | 4096 | yes (16 tracked envs) | -1.9043 | 0.4303 | outside | 2.0752 | 0.4335 | outside | -0.1709 | 0.0809 | outside |  |  |  |  |  |  |  |  |  |  |  |  |
| demohist | head | demostd | head | 4096 | yes (16 tracked envs) | 3.0273 | 0.6188 | outside | -2.9785 | 0.6188 | outside | -0.0488 | 0.0345 | inside | 0.708 | 0.3693 | inside | -0.0488 | 0.1145 | inside | 0.0244 | 0.3444 | inside | -3.6621 | 0.5842 | outside |
| demohist | head | acd3 | head | 4096 | yes (16 tracked envs) | 3.7109 | 0.6795 | outside | -3.2959 | 0.6778 | outside | -0.415 | 0.1005 | outside |  |  |  |  |  |  |  |  |  |  |  |  |
| demohist | head | std | head | 4096 | yes (16 tracked envs) | -1.416 | 0.6584 | outside | 1.7822 | 0.6578 | outside | -0.3662 | 0.0944 | outside |  |  |  |  |  |  |  |  |  |  |  |  |
| histstd | demo1 | std | demo1 | 4096 | yes (16 tracked envs) | -1.2207 | 0.5948 | outside | 3.6377 | 0.6486 | outside | -2.417 | 0.2936 | outside | -0.3906 | 0.4541 | inside | 3.0029 | 0.6251 | outside | 0.6836 | 0.4084 | inside | 0.3418 | 0.4968 | inside |
| histstd | demo1 | demohist | demo1 | 4096 | yes (16 tracked envs) | -1.5381 | 0.5941 | outside | 1.416 | 0.6297 | outside | 0.1221 | 0.2303 | inside | 0.1465 | 0.434 | inside | 1.2695 | 0.5948 | outside | 2.0264 | 0.3902 | outside | -2.0264 | 0.4842 | outside |
| histstd | demo1 | acd3 | demo1 | 4096 | yes (16 tracked envs) | 7.6172 | 0.6415 | outside | -5.2979 | 0.6947 | outside | -2.3193 | 0.3228 | outside | -1.123 | 0.4856 | outside | -1.9775 | 0.6693 | outside | -0.708 | 0.4454 | inside | -1.4893 | 0.5122 | outside |
| histstd | demo2 | std | demo2 | 4096 | yes (16 tracked envs) | -1.3672 | 0.568 | outside | 4.1748 | 0.6395 | outside | -2.8076 | 0.352 | outside | -0.2441 | 0.5305 | inside | 3.8818 | 0.639 | outside | 0.7568 | 0.4091 | inside | -0.2197 | 0.5151 | inside |
| histstd | demo2 | demohist | demo2 | 4096 | yes (16 tracked envs) | -1.1475 | 0.5473 | outside | 0.6592 | 0.5985 | inside | 0.4883 | 0.2583 | inside | -0.1465 | 0.4908 | inside | 1.4648 | 0.6026 | outside | 0.8057 | 0.3866 | outside | -1.4648 | 0.4829 | outside |
| histstd | demo2 | acd3 | demo2 | 4096 | yes (16 tracked envs) | 6.8115 | 0.5911 | outside | -2.5146 | 0.6899 | outside | -4.2969 | 0.4147 | outside | -0.1953 | 0.5327 | inside | -0.293 | 0.6669 | inside | -0.7324 | 0.4242 | inside | -1.2939 | 0.5396 | outside |
| histstd | aisle | std | aisle | 4096 | yes (16 tracked envs) | 0.7568 | 0.5689 | inside | 4.6875 | 0.6565 | outside | -5.4443 | 0.4852 | outside | -0.2441 | 0.3654 | inside | 4.9316 | 0.6642 | outside | 0.0 | 0.0 | inside | 0.0 | 0.0 | inside |
| histstd | aisle | demohist | aisle | 4096 | yes (16 tracked envs) | -3.54 | 0.5448 | outside | 2.3193 | 0.5965 | outside | 1.2207 | 0.3649 | outside | -0.2441 | 0.3798 | inside | 2.5635 | 0.6369 | outside | 0.0 | 0.0 | inside | 0.0 | 0.0 | inside |
| histstd | aisle | acd3 | aisle | 4096 | yes (16 tracked envs) | 12.0361 | 0.6422 | outside | -8.2764 | 0.7022 | outside | -3.7598 | 0.4774 | outside | -1.4648 | 0.4079 | outside | -6.8115 | 0.7333 | outside | 0.0 | 0.0 | inside | 0.0 | 0.0 | inside |
| histstd | rand | std | rand | 4096 | yes (16 tracked envs) | -1.2939 | 0.4103 | outside | 1.4404 | 0.4145 | outside | -0.1465 | 0.0913 | inside |  |  |  |  |  |  |  |  |  |  |  |  |
| histstd | rand | demohist | rand | 4096 | yes (16 tracked envs) | 0.6104 | 0.4319 | inside | -0.6348 | 0.4312 | inside | 0.0244 | 0.0646 | inside | -0.1465 | 0.2072 | inside | 0.2441 | 0.1465 | inside | -0.5859 | 0.3702 | inside | -0.1465 | 0.2607 | inside |
| histstd | rand | acd3 | rand | 4096 | yes (16 tracked envs) | 1.6602 | 0.46 | outside | -1.3428 | 0.4609 | outside | -0.3174 | 0.1118 | outside |  |  |  |  |  |  |  |  |  |  |  |  |
| histstd | head | std | head | 4096 | yes (16 tracked envs) | -3.2715 | 0.6146 | outside | 3.6377 | 0.6146 | outside | -0.3662 | 0.0944 | outside |  |  |  |  |  |  |  |  |  |  |  |  |
| histstd | head | demohist | head | 4096 | yes (16 tracked envs) | -1.8555 | 0.6609 | outside | 1.8555 | 0.6609 | outside | 0.0 | 0.0 | inside | -0.5127 | 0.3726 | inside | 0.3174 | 0.1359 | outside | -0.9521 | 0.3772 | outside | 3.0029 | 0.6231 | outside |
| histstd | head | acd3 | head | 4096 | yes (16 tracked envs) | 1.8555 | 0.5954 | outside | -1.4404 | 0.5932 | outside | -0.415 | 0.1005 | outside |  |  |  |  |  |  |  |  |  |  |  |  |
| demohist2 | demo1 | demohist | demo1 | 4096 | yes (16 tracked envs) | 1.4648 | 0.5422 | outside | -1.6846 | 0.5788 | outside | 0.2197 | 0.2057 | inside | -0.1221 | 0.4078 | inside | -0.9033 | 0.5884 | inside | 0.9033 | 0.3407 | outside | -1.5625 | 0.4456 | outside |
| demostd2 | demo1 | demostd | demo1 | 4096 | yes (16 tracked envs) | 2.0508 | 0.4958 | outside | -2.8564 | 0.5246 | outside | 0.8057 | 0.1739 | outside | 0.0244 | 0.3646 | inside | -2.4414 | 0.5037 | outside | 0.2197 | 0.3357 | inside | -0.6592 | 0.4048 | inside |
| demohist2 | demo1 | demostd2 | demo1 | 4096 | yes (16 tracked envs) | 0.9766 | 0.5766 | inside | -1.0986 | 0.6189 | inside | 0.1221 | 0.2277 | inside | 0.8545 | 0.4248 | outside | -1.6113 | 0.5875 | outside | -0.3906 | 0.3703 | inside | 0.0488 | 0.4709 | inside |
| demohist2 | demo1 | acd3 | demo1 | 4096 | yes (16 tracked envs) | 10.6201 | 0.6832 | outside | -8.3984 | 0.7475 | outside | -2.2217 | 0.3374 | outside | -1.3916 | 0.4647 | outside | -4.1504 | 0.7072 | outside | -1.8311 | 0.4694 | outside | -1.0254 | 0.5414 | inside |
| demostd2 | demo1 | acd3 | demo1 | 4096 | yes (16 tracked envs) | 9.6436 | 0.6797 | outside | -7.2998 | 0.7289 | outside | -2.3438 | 0.3162 | outside | -2.2461 | 0.4632 | outside | -2.5391 | 0.7074 | outside | -1.4404 | 0.471 | outside | -1.0742 | 0.5325 | outside |
| demohist2m | demo1 | demohist | demo1 | 4096 | yes (16 tracked envs) | 1.9531 | 0.5228 | outside | -1.7334 | 0.5502 | outside | -0.2197 | 0.1743 | inside | -0.2686 | 0.3899 | inside | -0.6836 | 0.527 | inside | 0.5859 | 0.322 | inside | -1.3672 | 0.4252 | outside |
| demostd2m | demo1 | demostd | demo1 | 4096 | yes (16 tracked envs) | 0.4883 | 0.4797 | inside | -0.8057 | 0.5032 | inside | 0.3174 | 0.1524 | outside | 0.2686 | 0.3513 | inside | -0.4395 | 0.4944 | inside | -0.293 | 0.3294 | inside | -0.3418 | 0.3982 | inside |
| demohist2m | demo1 | acd3 | demo1 | 4096 | yes (16 tracked envs) | 11.1084 | 0.6821 | outside | -8.4473 | 0.7328 | outside | -2.6611 | 0.3147 | outside | -1.5381 | 0.4671 | outside | -3.9307 | 0.69 | outside | -2.1484 | 0.4659 | outside | -0.8301 | 0.5314 | inside |
| demostd2m | demo1 | acd3 | demo1 | 4096 | yes (16 tracked envs) | 8.0811 | 0.6812 | outside | -5.249 | 0.7283 | outside | -2.832 | 0.3115 | outside | -2.002 | 0.4775 | outside | -0.5371 | 0.7076 | inside | -1.9531 | 0.4674 | outside | -0.7568 | 0.5343 | inside |
| demohist2 | demo2 | demohist | demo2 | 4096 | yes (16 tracked envs) | 1.7822 | 0.5061 | outside | -2.002 | 0.5527 | outside | 0.2197 | 0.2251 | inside | -0.415 | 0.474 | inside | -0.0488 | 0.608 | inside | -0.3418 | 0.34 | inside | -1.1963 | 0.4699 | outside |
| demostd2 | demo2 | demostd | demo2 | 4096 | yes (16 tracked envs) | 1.4648 | 0.4704 | outside | -2.2461 | 0.5086 | outside | 0.7812 | 0.2125 | outside | -0.7812 | 0.4099 | inside | -1.2451 | 0.5205 | outside | 0.1465 | 0.3088 | inside | -0.3662 | 0.4179 | inside |
| demohist2 | demo2 | demostd2 | demo2 | 4096 | yes (16 tracked envs) | 0.7812 | 0.5315 | inside | -0.5615 | 0.5741 | inside | -0.2197 | 0.2405 | inside | 1.3428 | 0.453 | outside | -1.0254 | 0.6223 | inside | -0.9521 | 0.3643 | outside | 0.0732 | 0.4741 | inside |
| demohist2 | demo2 | acd3 | demo2 | 4096 | yes (16 tracked envs) | 9.7412 | 0.6245 | outside | -5.1758 | 0.7206 | outside | -4.5654 | 0.406 | outside | -0.4639 | 0.5242 | inside | -1.8066 | 0.732 | outside | -1.8799 | 0.4351 | outside | -1.0254 | 0.5302 | inside |
| demostd2 | demo2 | acd3 | demo2 | 4096 | yes (16 tracked envs) | 8.96 | 0.6158 | outside | -4.6143 | 0.7069 | outside | -4.3457 | 0.3999 | outside | -1.8066 | 0.5079 | outside | -0.7812 | 0.7118 | inside | -0.9277 | 0.4352 | outside | -1.0986 | 0.5296 | outside |
| demohist2 | aisle | demohist | aisle | 4096 | yes (16 tracked envs) | 1.7578 | 0.4924 | outside | -1.4648 | 0.5509 | outside | -0.293 | 0.2629 | inside | -0.8789 | 0.3363 | outside | -0.5859 | 0.603 | inside | 0.0 | 0.0 | inside | 0.0 | 0.0 | inside |
| demostd2 | aisle | demostd | aisle | 4096 | yes (16 tracked envs) | 2.5146 | 0.4478 | outside | -3.6133 | 0.4826 | outside | 1.0986 | 0.2323 | outside | -0.6104 | 0.2547 | outside | -3.0029 | 0.5083 | outside | 0.0 | 0.0 | inside | 0.0 | 0.0 | inside |
| demohist2 | aisle | demostd2 | aisle | 4096 | yes (16 tracked envs) | 1.0254 | 0.4893 | outside | -1.001 | 0.5603 | inside | -0.0244 | 0.2858 | inside | 1.4648 | 0.3021 | outside | -2.4658 | 0.5983 | outside | 0.0 | 0.0 | inside | 0.0 | 0.0 | inside |
| demohist2 | aisle | acd3 | aisle | 4096 | yes (16 tracked envs) | 17.334 | 0.6735 | outside | -12.0605 | 0.7278 | outside | -5.2734 | 0.4506 | outside | -2.0996 | 0.3954 | outside | -9.9609 | 0.7819 | outside | 0.0 | 0.0 | inside | 0.0 | 0.0 | inside |
| demostd2 | aisle | acd3 | aisle | 4096 | yes (16 tracked envs) | 16.3086 | 0.7047 | outside | -11.0596 | 0.7402 | outside | -5.249 | 0.4434 | outside | -3.5645 | 0.3882 | outside | -7.4951 | 0.7844 | outside | 0.0 | 0.0 | inside | 0.0 | 0.0 | inside |
| demohist2 | rand | demohist | rand | 4096 | yes (16 tracked envs) | 1.1719 | 0.4096 | outside | -1.123 | 0.4111 | outside | -0.0488 | 0.0345 | inside | 0.1953 | 0.2211 | inside | -0.0244 | 0.1007 | inside | -1.1475 | 0.3642 | outside | -0.1465 | 0.2514 | inside |
| demostd2 | rand | demostd | rand | 4096 | yes (16 tracked envs) | 1.0986 | 0.3691 | outside | -1.0742 | 0.3683 | outside | -0.0244 | 0.0244 | inside | -0.3174 | 0.2197 | inside | 0.0244 | 0.0946 | inside | -0.2441 | 0.3049 | inside | -0.5371 | 0.2489 | outside |
| demohist2 | rand | demostd2 | rand | 4096 | yes (16 tracked envs) | -0.0732 | 0.4388 | inside | 0.0977 | 0.4409 | inside | -0.0244 | 0.0423 | inside | 0.3174 | 0.2379 | inside | -0.0732 | 0.1221 | inside | -0.6592 | 0.3562 | inside | 0.5127 | 0.2815 | inside |
| demohist2 | rand | acd3 | rand | 4096 | yes (16 tracked envs) | 2.2217 | 0.4816 | outside | -1.8311 | 0.4807 | outside | -0.3906 | 0.0975 | outside |  |  |  |  |  |  |  |  |  |  |  |  |
| demostd2 | rand | acd3 | rand | 4096 | yes (16 tracked envs) | 2.2949 | 0.4796 | outside | -1.9287 | 0.4769 | outside | -0.3662 | 0.1063 | outside |  |  |  |  |  |  |  |  |  |  |  |  |
| demohistdyn | demo1 | demohist | demo1 | 4096 | yes (16 tracked envs) | -3.7598 | 0.5738 | outside | 4.5166 | 0.5974 | outside | -0.7568 | 0.1705 | outside | -0.4883 | 0.4354 | inside | 3.833 | 0.5835 | outside | 1.5869 | 0.3621 | outside | -0.415 | 0.4778 | inside |
| demohistdyn | demo1 | acd3 | demo1 | 4096 | yes (16 tracked envs) | 5.3955 | 0.6638 | outside | -2.1973 | 0.7085 | outside | -3.1982 | 0.3018 | outside | -1.7578 | 0.4701 | outside | 0.5859 | 0.6923 | inside | -1.1475 | 0.4812 | outside | 0.1221 | 0.5377 | inside |
| demohistdyn | demo2 | demohist | demo2 | 4096 | yes (16 tracked envs) | -2.6855 | 0.5355 | outside | 3.7354 | 0.5639 | outside | -1.0498 | 0.1868 | outside | -0.3418 | 0.4895 | inside | 3.8818 | 0.5844 | outside | 0.3662 | 0.3613 | inside | -0.1709 | 0.5034 | inside |
| demohistdyn | demo2 | acd3 | demo2 | 4096 | yes (16 tracked envs) | 5.2734 | 0.6024 | outside | 0.5615 | 0.6806 | inside | -5.835 | 0.3775 | outside | -0.3906 | 0.5237 | inside | 2.124 | 0.6971 | outside | -1.1719 | 0.4551 | outside | 0.0 | 0.5557 | inside |
| demohistdyn | aisle | demohist | aisle | 4096 | yes (16 tracked envs) | -4.0283 | 0.4994 | outside | 5.2979 | 0.5522 | outside | -1.2695 | 0.2553 | outside | -1.2695 | 0.3762 | outside | 6.5674 | 0.627 | outside | 0.0 | 0.0 | inside | 0.0 | 0.0 | inside |
| demohistdyn | aisle | acd3 | aisle | 4096 | yes (16 tracked envs) | 11.5479 | 0.6856 | outside | -5.2979 | 0.7225 | outside | -6.25 | 0.4327 | outside | -2.4902 | 0.4038 | outside | -2.8076 | 0.7682 | outside | 0.0 | 0.0 | inside | 0.0 | 0.0 | inside |
| demohistdyn | rand | demohist | rand | 4096 | yes (16 tracked envs) | -1.2207 | 0.4418 | outside | 1.2207 | 0.4431 | outside | 0.0 | 0.0488 | inside | 0.6836 | 0.2488 | outside | 0.4639 | 0.1523 | outside | 0.708 | 0.3709 | inside | -0.6348 | 0.2673 | outside |
| demohistdyn | rand | acd3 | rand | 4096 | yes (16 tracked envs) | -0.1709 | 0.4926 | inside | 0.5127 | 0.4937 | inside | -0.3418 | 0.1091 | outside |  |  |  |  |  |  |  |  |  |  |  |  |
| acd3 | demo1c08 | acd3 | demo1 | 4096 | different start episodes |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
| std | demo1c08 | std | demo1 | 4096 | different start episodes |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
| demohist | demo1c08 | demohist | demo1 | 4096 | different start episodes |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
| std | demo1c08 | acd3 | demo1c08 | 4096 | yes (16 tracked envs) | 9.2285 | 0.6575 | outside | -6.7871 | 0.737 | outside | -2.4414 | 0.3949 | outside | 0.1709 | 0.4639 | inside | -5.7617 | 0.7002 | outside | -1.001 | 0.4662 | outside | -0.1953 | 0.5248 | inside |
| demohist | demo1c08 | acd3 | demo1c08 | 4096 | yes (16 tracked envs) | 7.4219 | 0.6764 | outside | -3.1494 | 0.7402 | outside | -4.2725 | 0.3634 | outside | -0.7568 | 0.4599 | inside | -2.9541 | 0.7175 | outside | -0.8057 | 0.502 | inside | 1.3672 | 0.5334 | outside |
| demohist | demo1c08 | std | demo1c08 | 4096 | yes (16 tracked envs) | -1.8066 | 0.6073 | outside | 3.6377 | 0.6595 | outside | -1.8311 | 0.2926 | outside | -0.9277 | 0.4618 | outside | 2.8076 | 0.631 | outside | 0.1953 | 0.4201 | inside | 1.5625 | 0.489 | outside |

## 3. 회귀 확인 (같은 체크포인트, 같은 seed, 병합 코드)


| 실행 | 기준 | 같은 에피소드 | 에피소드 | 결과 다른 수 | 길이 다른 수 | 성공 차 %p | 성공 SE | 성공 2SE | from_success_changed | from_collision_changed | from_oob_changed | from_timeout_changed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| e_acd3_rand_chk2 | e_acd3_rand | yes (16 tracked envs) | 4096 | 18 | 282 | -0.0244 | 0.0946 | inside | 8 | 7 | 0 | 3 |

## 4. 학습 설정

걸린 시간은 queue_status.txt의 wall_s.

| 실행 | 설정 출처 | 걸린 시간 s | t_ctrl과 다른 설정 |
|---|---|---|---|
| t_ctrl | run_config.json | 2152 |  |
| t_demo | run_config.json | 2276 | arena_spec.block_m=0.0 (ctrl -); arena_spec.block_v_mps=0.02 (ctrl -); arena_spec.ghost_headon_prob=0.25 (ctrl -); arena_spec.ghost_pairs=0 (ctrl -); arena_spec.ghost_scenario_prob=0.25 (ctrl -); arena_spec.mover_cross_prob=0.5 (ctrl -); arena_spec.mover_kinds= (ctrl -); arena_spec.near_dyn_m=0.0 (ctrl -); arena_spec.obstacle_demo_aisle_min_m=0.3 (ctrl -); arena_spec.obstacle_demo_aisle_prob=0.7 (ctrl -); arena_spec.obstacle_demo_counts=[1, 2] (ctrl -); arena_spec.obstacle_demo_gap_m=[0.13, 0.25] (ctrl -); arena_spec.obstacle_demo_keep_path=1 (ctrl -); arena_spec.obstacle_demo_share=0.5 (ctrl -); arena_spec.robot_cleaner_prob=0.5 (ctrl -); arena_spec.robot_cleaner_wait_s=10.0 (ctrl -); arena_spec.robot_cleaner_yield_m=0.3 (ctrl -); arena_spec.robot_min=0 (ctrl -); arena_spec.robot_scenario= (ctrl -); arena_spec.w_block=0.0 (ctrl -); arena_spec.w_near_dyn=0.0 (ctrl -); arena_spec.w_yield=0.0 (ctrl -); arena_spec.yield_m=0.0 (ctrl -); caps.enabled=False (ctrl -); caps.sigma=0.0 (ctrl -); caps.spatial=0.0 (ctrl -); caps.temporal=0.0 (ctrl -); symmetry.mode=off (ctrl -); task_spec.cmd_delay_steps_range=[0, 0] (ctrl -); task_spec.cmd_gain_v_range=[1.0, 1.0] (ctrl -); task_spec.cmd_gain_w_range=[1.0, 1.0] (ctrl -); task_spec.cmd_lag_tau_range=[0.0, 0.0] (ctrl -) |
| t_demo_hist | run_config.json | 2276 | arena_spec.block_m=0.0 (ctrl -); arena_spec.block_v_mps=0.02 (ctrl -); arena_spec.ghost_headon_prob=0.25 (ctrl -); arena_spec.ghost_pairs=0 (ctrl -); arena_spec.ghost_scenario_prob=0.25 (ctrl -); arena_spec.mover_cross_prob=0.5 (ctrl -); arena_spec.mover_kinds= (ctrl -); arena_spec.near_dyn_m=0.0 (ctrl -); arena_spec.obstacle_demo_aisle_min_m=0.3 (ctrl -); arena_spec.obstacle_demo_aisle_prob=0.7 (ctrl -); arena_spec.obstacle_demo_counts=[1, 2] (ctrl -); arena_spec.obstacle_demo_gap_m=[0.13, 0.25] (ctrl -); arena_spec.obstacle_demo_keep_path=1 (ctrl -); arena_spec.obstacle_demo_share=0.5 (ctrl -); arena_spec.robot_cleaner_prob=0.5 (ctrl -); arena_spec.robot_cleaner_wait_s=10.0 (ctrl -); arena_spec.robot_cleaner_yield_m=0.3 (ctrl -); arena_spec.robot_min=0 (ctrl -); arena_spec.robot_scenario= (ctrl -); arena_spec.w_block=0.0 (ctrl -); arena_spec.w_near_dyn=0.0 (ctrl -); arena_spec.w_yield=0.0 (ctrl -); arena_spec.yield_m=0.0 (ctrl -); caps.enabled=False (ctrl -); caps.sigma=0.0 (ctrl -); caps.spatial=0.0 (ctrl -); caps.temporal=0.0 (ctrl -); init=D:/isaac/runs/store_arena/n26/init_acd3_hist3.pt (ctrl D:/isaac/runs/store_arena/acd3/t_wall/model_1745.pt); init_std=[0.3, 0.3] (ctrl [0.0, 0.0]); symmetry.mode=off (ctrl -); task_spec.cmd_delay_steps_range=[0, 0] (ctrl -); task_spec.cmd_gain_v_range=[1.0, 1.0] (ctrl -); task_spec.cmd_gain_w_range=[1.0, 1.0] (ctrl -); task_spec.cmd_lag_tau_range=[0.0, 0.0] (ctrl -); task_spec.lidar_frames=3 (ctrl 1) |
| t_demo_hist2 | run_config.json | 4510 | agent.max_iterations=600 (ctrl 300); arena_spec.block_m=0.0 (ctrl -); arena_spec.block_v_mps=0.02 (ctrl -); arena_spec.ghost_headon_prob=0.25 (ctrl -); arena_spec.ghost_pairs=0 (ctrl -); arena_spec.ghost_scenario_prob=0.25 (ctrl -); arena_spec.mover_cross_prob=0.5 (ctrl -); arena_spec.mover_kinds= (ctrl -); arena_spec.near_dyn_m=0.0 (ctrl -); arena_spec.obstacle_demo_aisle_min_m=0.3 (ctrl -); arena_spec.obstacle_demo_aisle_prob=0.7 (ctrl -); arena_spec.obstacle_demo_counts=[1, 2] (ctrl -); arena_spec.obstacle_demo_gap_m=[0.13, 0.25] (ctrl -); arena_spec.obstacle_demo_keep_path=1 (ctrl -); arena_spec.obstacle_demo_share=0.5 (ctrl -); arena_spec.robot_cleaner_prob=0.5 (ctrl -); arena_spec.robot_cleaner_wait_s=10.0 (ctrl -); arena_spec.robot_cleaner_yield_m=0.3 (ctrl -); arena_spec.robot_min=0 (ctrl -); arena_spec.robot_scenario= (ctrl -); arena_spec.w_block=0.0 (ctrl -); arena_spec.w_near_dyn=0.0 (ctrl -); arena_spec.w_yield=0.0 (ctrl -); arena_spec.yield_m=0.0 (ctrl -); caps.enabled=False (ctrl -); caps.sigma=0.0 (ctrl -); caps.spatial=0.0 (ctrl -); caps.temporal=0.0 (ctrl -); init= (ctrl D:/isaac/runs/store_arena/acd3/t_wall/model_1745.pt); max_iterations=600 (ctrl 300); symmetry.mode=off (ctrl -); task_spec.cmd_delay_steps_range=[0, 0] (ctrl -); task_spec.cmd_gain_v_range=[1.0, 1.0] (ctrl -); task_spec.cmd_gain_w_range=[1.0, 1.0] (ctrl -); task_spec.cmd_lag_tau_range=[0.0, 0.0] (ctrl -); task_spec.lidar_frames=3 (ctrl 1) |
| t_demo_hist_dyn | run_config.json | 2254 | arena_spec.block_m=0.0 (ctrl -); arena_spec.block_v_mps=0.02 (ctrl -); arena_spec.ghost_headon_prob=0.25 (ctrl -); arena_spec.ghost_pairs=0 (ctrl -); arena_spec.ghost_scenario_prob=0.25 (ctrl -); arena_spec.mover_cross_prob=0.5 (ctrl -); arena_spec.mover_kinds= (ctrl -); arena_spec.near_dyn_m=0.1 (ctrl -); arena_spec.obstacle_demo_aisle_min_m=0.3 (ctrl -); arena_spec.obstacle_demo_aisle_prob=0.7 (ctrl -); arena_spec.obstacle_demo_counts=[1, 2] (ctrl -); arena_spec.obstacle_demo_gap_m=[0.13, 0.25] (ctrl -); arena_spec.obstacle_demo_keep_path=1 (ctrl -); arena_spec.obstacle_demo_share=0.5 (ctrl -); arena_spec.robot_cleaner_prob=0.5 (ctrl -); arena_spec.robot_cleaner_wait_s=10.0 (ctrl -); arena_spec.robot_cleaner_yield_m=0.3 (ctrl -); arena_spec.robot_min=0 (ctrl -); arena_spec.robot_scenario= (ctrl -); arena_spec.w_block=0.0 (ctrl -); arena_spec.w_near_dyn=-1.0 (ctrl -); arena_spec.w_yield=0.0 (ctrl -); arena_spec.yield_m=0.0 (ctrl -); caps.enabled=False (ctrl -); caps.sigma=0.0 (ctrl -); caps.spatial=0.0 (ctrl -); caps.temporal=0.0 (ctrl -); init=D:/isaac/runs/store_arena/n26/init_acd3_hist3.pt (ctrl D:/isaac/runs/store_arena/acd3/t_wall/model_1745.pt); init_std=[0.3, 0.3] (ctrl [0.0, 0.0]); symmetry.mode=off (ctrl -); task_spec.cmd_delay_steps_range=[0, 0] (ctrl -); task_spec.cmd_gain_v_range=[1.0, 1.0] (ctrl -); task_spec.cmd_gain_w_range=[1.0, 1.0] (ctrl -); task_spec.cmd_lag_tau_range=[0.0, 0.0] (ctrl -); task_spec.lidar_frames=3 (ctrl 1) |
| t_demo_std | run_config.json | 2244 | arena_spec.block_m=0.0 (ctrl -); arena_spec.block_v_mps=0.02 (ctrl -); arena_spec.ghost_headon_prob=0.25 (ctrl -); arena_spec.ghost_pairs=0 (ctrl -); arena_spec.ghost_scenario_prob=0.25 (ctrl -); arena_spec.mover_cross_prob=0.5 (ctrl -); arena_spec.mover_kinds= (ctrl -); arena_spec.near_dyn_m=0.0 (ctrl -); arena_spec.obstacle_demo_aisle_min_m=0.3 (ctrl -); arena_spec.obstacle_demo_aisle_prob=0.7 (ctrl -); arena_spec.obstacle_demo_counts=[1, 2] (ctrl -); arena_spec.obstacle_demo_gap_m=[0.13, 0.25] (ctrl -); arena_spec.obstacle_demo_keep_path=1 (ctrl -); arena_spec.obstacle_demo_share=0.5 (ctrl -); arena_spec.robot_cleaner_prob=0.5 (ctrl -); arena_spec.robot_cleaner_wait_s=10.0 (ctrl -); arena_spec.robot_cleaner_yield_m=0.3 (ctrl -); arena_spec.robot_min=0 (ctrl -); arena_spec.robot_scenario= (ctrl -); arena_spec.w_block=0.0 (ctrl -); arena_spec.w_near_dyn=0.0 (ctrl -); arena_spec.w_yield=0.0 (ctrl -); arena_spec.yield_m=0.0 (ctrl -); caps.enabled=False (ctrl -); caps.sigma=0.0 (ctrl -); caps.spatial=0.0 (ctrl -); caps.temporal=0.0 (ctrl -); init_std=[0.3, 0.3] (ctrl [0.0, 0.0]); symmetry.mode=off (ctrl -); task_spec.cmd_delay_steps_range=[0, 0] (ctrl -); task_spec.cmd_gain_v_range=[1.0, 1.0] (ctrl -); task_spec.cmd_gain_w_range=[1.0, 1.0] (ctrl -); task_spec.cmd_lag_tau_range=[0.0, 0.0] (ctrl -) |
| t_demo_std2 | run_config.json | 4474 | agent.max_iterations=600 (ctrl 300); arena_spec.block_m=0.0 (ctrl -); arena_spec.block_v_mps=0.02 (ctrl -); arena_spec.ghost_headon_prob=0.25 (ctrl -); arena_spec.ghost_pairs=0 (ctrl -); arena_spec.ghost_scenario_prob=0.25 (ctrl -); arena_spec.mover_cross_prob=0.5 (ctrl -); arena_spec.mover_kinds= (ctrl -); arena_spec.near_dyn_m=0.0 (ctrl -); arena_spec.obstacle_demo_aisle_min_m=0.3 (ctrl -); arena_spec.obstacle_demo_aisle_prob=0.7 (ctrl -); arena_spec.obstacle_demo_counts=[1, 2] (ctrl -); arena_spec.obstacle_demo_gap_m=[0.13, 0.25] (ctrl -); arena_spec.obstacle_demo_keep_path=1 (ctrl -); arena_spec.obstacle_demo_share=0.5 (ctrl -); arena_spec.robot_cleaner_prob=0.5 (ctrl -); arena_spec.robot_cleaner_wait_s=10.0 (ctrl -); arena_spec.robot_cleaner_yield_m=0.3 (ctrl -); arena_spec.robot_min=0 (ctrl -); arena_spec.robot_scenario= (ctrl -); arena_spec.w_block=0.0 (ctrl -); arena_spec.w_near_dyn=0.0 (ctrl -); arena_spec.w_yield=0.0 (ctrl -); arena_spec.yield_m=0.0 (ctrl -); caps.enabled=False (ctrl -); caps.sigma=0.0 (ctrl -); caps.spatial=0.0 (ctrl -); caps.temporal=0.0 (ctrl -); init= (ctrl D:/isaac/runs/store_arena/acd3/t_wall/model_1745.pt); max_iterations=600 (ctrl 300); symmetry.mode=off (ctrl -); task_spec.cmd_delay_steps_range=[0, 0] (ctrl -); task_spec.cmd_gain_v_range=[1.0, 1.0] (ctrl -); task_spec.cmd_gain_w_range=[1.0, 1.0] (ctrl -); task_spec.cmd_lag_tau_range=[0.0, 0.0] (ctrl -) |
| t_dyn | run_config.json | 2100 | arena_spec.block_m=0.0 (ctrl -); arena_spec.block_v_mps=0.02 (ctrl -); arena_spec.near_dyn_m=0.1 (ctrl -); arena_spec.robot_scenario= (ctrl -); arena_spec.w_block=0.0 (ctrl -); arena_spec.w_near_dyn=-1.0 (ctrl -); arena_spec.w_yield=0.0 (ctrl -); arena_spec.yield_m=0.0 (ctrl -); task_spec.cmd_delay_steps_range=[0, 0] (ctrl -); task_spec.cmd_gain_v_range=[1.0, 1.0] (ctrl -); task_spec.cmd_gain_w_range=[1.0, 1.0] (ctrl -); task_spec.cmd_lag_tau_range=[0.0, 0.0] (ctrl -) |
| t_hist_std | run_config.json | 2122 | arena_spec.block_m=0.0 (ctrl -); arena_spec.block_v_mps=0.02 (ctrl -); arena_spec.ghost_headon_prob=0.25 (ctrl -); arena_spec.ghost_pairs=0 (ctrl -); arena_spec.ghost_scenario_prob=0.25 (ctrl -); arena_spec.mover_cross_prob=0.0 (ctrl -); arena_spec.mover_kinds= (ctrl -); arena_spec.near_dyn_m=0.0 (ctrl -); arena_spec.obstacle_demo_aisle_min_m=0.3 (ctrl -); arena_spec.obstacle_demo_aisle_prob=0.7 (ctrl -); arena_spec.obstacle_demo_counts=[1, 2] (ctrl -); arena_spec.obstacle_demo_gap_m=[0.13, 0.25] (ctrl -); arena_spec.obstacle_demo_keep_path=1 (ctrl -); arena_spec.obstacle_demo_share=0.0 (ctrl -); arena_spec.robot_cleaner_prob=0.0 (ctrl -); arena_spec.robot_cleaner_wait_s=10.0 (ctrl -); arena_spec.robot_cleaner_yield_m=0.3 (ctrl -); arena_spec.robot_min=0 (ctrl -); arena_spec.robot_scenario= (ctrl -); arena_spec.w_block=0.0 (ctrl -); arena_spec.w_near_dyn=0.0 (ctrl -); arena_spec.w_yield=0.0 (ctrl -); arena_spec.yield_m=0.0 (ctrl -); caps.enabled=False (ctrl -); caps.sigma=0.0 (ctrl -); caps.spatial=0.0 (ctrl -); caps.temporal=0.0 (ctrl -); init=D:/isaac/runs/store_arena/n26/init_acd3_hist3.pt (ctrl D:/isaac/runs/store_arena/acd3/t_wall/model_1745.pt); init_std=[0.3, 0.3] (ctrl [0.0, 0.0]); symmetry.mode=off (ctrl -); task_spec.cmd_delay_steps_range=[0, 0] (ctrl -); task_spec.cmd_gain_v_range=[1.0, 1.0] (ctrl -); task_spec.cmd_gain_w_range=[1.0, 1.0] (ctrl -); task_spec.cmd_lag_tau_range=[0.0, 0.0] (ctrl -); task_spec.lidar_frames=3 (ctrl 1) |
| t_std | run_config.json | 2061 | init_std=[0.3, 0.3] (ctrl [0.0, 0.0]) |

## 4b. 학습 곡선

반복은 이 학습 안에서의 반복(0은 시작). metrics.csv에는 보상과 평균 에피소드 길이가 없어 성공·충돌·시간 초과율과 성공 시간만 적는다. std는 저장된 model_<누적 반복>.pt의 distribution.std_param.

| 실행 | 반복 | 누적 반복 | 성공률 | 충돌률 | 시간 초과율 | 성공 시간 s | std v | std w |
|---|---|---|---|---|---|---|---|---|
| t_ctrl | 0 | 1745 |  |  |  |  | 0.0566 | 0.0294 |
| t_ctrl | 5 | 1750 | 0.7557 | 0.153 | 0.0913 | 25.953 | 0.0587 | 0.0314 |
| t_ctrl | 55 | 1800 | 0.7859 | 0.2117 | 0.0024 | 11.995 | 0.0579 | 0.0312 |
| t_ctrl | 105 | 1850 | 0.7604 | 0.2396 | 0.0 | 12.613 | 0.0586 | 0.0302 |
| t_ctrl | 155 | 1900 | 0.7705 | 0.2295 | 0.0 | 11.451 | 0.0578 | 0.0279 |
| t_ctrl | 205 | 1950 | 0.7527 | 0.2451 | 0.0022 | 12.251 | 0.0571 | 0.027 |
| t_ctrl | 255 | 2000 | 0.7373 | 0.2581 | 0.0046 | 12.539 | 0.0554 | 0.0248 |
| t_ctrl | 299 | 2044 | 0.7372 | 0.2579 | 0.0049 | 12.22 | 0.0501 | 0.025 |
| t_demo | 0 | 1745 |  |  |  |  | 0.0566 | 0.0294 |
| t_demo | 5 | 1750 | 0.6647 | 0.252 | 0.0833 | 25.349 | 0.0592 | 0.0316 |
| t_demo | 55 | 1800 | 0.6975 | 0.2888 | 0.0136 | 12.69 | 0.0584 | 0.0302 |
| t_demo | 105 | 1850 | 0.6475 | 0.3325 | 0.02 | 11.488 | 0.0577 | 0.0271 |
| t_demo | 155 | 1900 | 0.6295 | 0.3634 | 0.0071 | 12.77 | 0.0554 | 0.0245 |
| t_demo | 205 | 1950 | 0.5957 | 0.3901 | 0.0142 | 11.744 | 0.0497 | 0.0217 |
| t_demo | 255 | 2000 | 0.5975 | 0.3899 | 0.0127 | 12.046 | 0.046 | 0.0217 |
| t_demo | 299 | 2044 | 0.6111 | 0.3768 | 0.0121 | 12.099 | 0.043 | 0.0195 |
| t_demo_hist | 0 | 0 |  |  |  |  | 0.3 | 0.3 |
| t_demo_hist | 1750 | 1750 |  |  |  |  | 0.2761 | 0.2813 |
| t_demo_hist | 1800 | 1800 |  |  |  |  | 0.201 | 0.1837 |
| t_demo_hist | 1850 | 1850 |  |  |  |  | 0.186 | 0.1447 |
| t_demo_hist | 1900 | 1900 |  |  |  |  | 0.1798 | 0.116 |
| t_demo_hist | 1950 | 1950 |  |  |  |  | 0.1716 | 0.0991 |
| t_demo_hist | 2000 | 2000 |  |  |  |  | 0.1607 | 0.0851 |
| t_demo_hist | 2044 | 2044 |  |  |  |  | 0.1556 | 0.0805 |
| t_demo_hist2 | 0 | 0 |  |  |  |  |  |  |
| t_demo_hist2 | 2050 | 2050 |  |  |  |  | 0.1538 | 0.0795 |
| t_demo_hist2 | 2100 | 2100 |  |  |  |  | 0.1468 | 0.073 |
| t_demo_hist2 | 2150 | 2150 |  |  |  |  | 0.1434 | 0.0692 |
| t_demo_hist2 | 2200 | 2200 |  |  |  |  | 0.1386 | 0.064 |
| t_demo_hist2 | 2250 | 2250 |  |  |  |  | 0.1321 | 0.0594 |
| t_demo_hist2 | 2300 | 2300 |  |  |  |  | 0.123 | 0.0519 |
| t_demo_hist2 | 2350 | 2350 |  |  |  |  | 0.118 | 0.0499 |
| t_demo_hist2 | 2400 | 2400 |  |  |  |  | 0.1108 | 0.0476 |
| t_demo_hist2 | 2450 | 2450 |  |  |  |  | 0.1073 | 0.0487 |
| t_demo_hist2 | 2500 | 2500 |  |  |  |  | 0.1044 | 0.0441 |
| t_demo_hist2 | 2550 | 2550 |  |  |  |  | 0.103 | 0.0451 |
| t_demo_hist2 | 2600 | 2600 |  |  |  |  | 0.096 | 0.0441 |
| t_demo_hist2 | 2643 | 2643 |  |  |  |  | 0.092 | 0.0405 |
| t_demo_hist_dyn | 0 | 0 |  |  |  |  | 0.3 | 0.3 |
| t_demo_hist_dyn | 1750 | 1750 |  |  |  |  | 0.2783 | 0.2869 |
| t_demo_hist_dyn | 1800 | 1800 |  |  |  |  | 0.2108 | 0.1991 |
| t_demo_hist_dyn | 1850 | 1850 |  |  |  |  | 0.2046 | 0.1462 |
| t_demo_hist_dyn | 1900 | 1900 |  |  |  |  | 0.211 | 0.1221 |
| t_demo_hist_dyn | 1950 | 1950 |  |  |  |  | 0.2117 | 0.1028 |
| t_demo_hist_dyn | 2000 | 2000 |  |  |  |  | 0.1991 | 0.0876 |
| t_demo_hist_dyn | 2044 | 2044 |  |  |  |  | 0.1888 | 0.079 |
| t_demo_std | 0 | 1745 |  |  |  |  | 0.3 | 0.3 |
| t_demo_std | 5 | 1750 | 0.6674 | 0.2421 | 0.0905 | 24.204 | 0.2717 | 0.2819 |
| t_demo_std | 55 | 1800 | 0.6289 | 0.3667 | 0.0044 | 12.049 | 0.2069 | 0.1944 |
| t_demo_std | 105 | 1850 | 0.6637 | 0.3341 | 0.0022 | 11.17 | 0.181 | 0.1426 |
| t_demo_std | 155 | 1900 | 0.6892 | 0.3087 | 0.0021 | 12.028 | 0.176 | 0.1092 |
| t_demo_std | 205 | 1950 | 0.6696 | 0.3282 | 0.0022 | 11.685 | 0.1698 | 0.0828 |
| t_demo_std | 255 | 2000 | 0.6231 | 0.3747 | 0.0022 | 12.137 | 0.1661 | 0.0706 |
| t_demo_std | 299 | 2044 | 0.7029 | 0.2927 | 0.0044 | 12.678 | 0.1564 | 0.0588 |
| t_demo_std2 | 0 | 0 |  |  |  |  |  |  |
| t_demo_std2 | 2050 | 2050 |  |  |  |  | 0.1526 | 0.058 |
| t_demo_std2 | 2100 | 2100 |  |  |  |  | 0.1421 | 0.0504 |
| t_demo_std2 | 2150 | 2150 |  |  |  |  | 0.1305 | 0.0435 |
| t_demo_std2 | 2200 | 2200 |  |  |  |  | 0.1271 | 0.0388 |
| t_demo_std2 | 2250 | 2250 |  |  |  |  | 0.1249 | 0.0348 |
| t_demo_std2 | 2300 | 2300 |  |  |  |  | 0.1208 | 0.0313 |
| t_demo_std2 | 2350 | 2350 |  |  |  |  | 0.1258 | 0.029 |
| t_demo_std2 | 2400 | 2400 |  |  |  |  | 0.1243 | 0.0291 |
| t_demo_std2 | 2450 | 2450 |  |  |  |  | 0.1186 | 0.0264 |
| t_demo_std2 | 2500 | 2500 |  |  |  |  | 0.1188 | 0.0237 |
| t_demo_std2 | 2550 | 2550 |  |  |  |  | 0.1184 | 0.0235 |
| t_demo_std2 | 2600 | 2600 |  |  |  |  | 0.1188 | 0.0217 |
| t_demo_std2 | 2643 | 2643 |  |  |  |  | 0.1209 | 0.0208 |
| t_dyn | 0 | 1745 |  |  |  |  | 0.0566 | 0.0294 |
| t_dyn | 5 | 1750 | 0.7528 | 0.1542 | 0.093 | 25.819 | 0.059 | 0.0318 |
| t_dyn | 55 | 1800 | 0.778 | 0.2197 | 0.0023 | 12.284 | 0.0596 | 0.0334 |
| t_dyn | 105 | 1850 | 0.723 | 0.2723 | 0.0047 | 12.674 | 0.0604 | 0.0343 |
| t_dyn | 155 | 1900 | 0.7923 | 0.2026 | 0.0051 | 12.088 | 0.0668 | 0.0345 |
| t_dyn | 205 | 1950 | 0.7465 | 0.2535 | 0.0 | 11.392 | 0.0678 | 0.0336 |
| t_dyn | 255 | 2000 | 0.7637 | 0.2338 | 0.0025 | 12.543 | 0.0654 | 0.0324 |
| t_dyn | 299 | 2044 | 0.7286 | 0.2641 | 0.0073 | 12.127 | 0.0696 | 0.0312 |
| t_hist_std | 0 | 0 |  |  |  |  | 0.3 | 0.3 |
| t_hist_std | 1750 | 1750 |  |  |  |  | 0.2722 | 0.2756 |
| t_hist_std | 1800 | 1800 |  |  |  |  | 0.1994 | 0.1875 |
| t_hist_std | 1850 | 1850 |  |  |  |  | 0.1771 | 0.1559 |
| t_hist_std | 1900 | 1900 |  |  |  |  | 0.1616 | 0.1327 |
| t_hist_std | 1950 | 1950 |  |  |  |  | 0.1509 | 0.1171 |
| t_hist_std | 2000 | 2000 |  |  |  |  | 0.1459 | 0.1141 |
| t_hist_std | 2044 | 2044 |  |  |  |  | 0.1417 | 0.1045 |
| t_std | 0 | 1745 |  |  |  |  | 0.3 | 0.3 |
| t_std | 5 | 1750 | 0.7462 | 0.1619 | 0.0919 | 25.668 | 0.2688 | 0.2727 |
| t_std | 55 | 1800 | 0.7949 | 0.2051 | 0.0 | 12.015 | 0.1971 | 0.1814 |
| t_std | 105 | 1850 | 0.7664 | 0.2313 | 0.0023 | 12.982 | 0.1664 | 0.1478 |
| t_std | 155 | 1900 | 0.7884 | 0.2116 | 0.0 | 11.681 | 0.148 | 0.1274 |
| t_std | 205 | 1950 | 0.7604 | 0.2396 | 0.0 | 11.337 | 0.14 | 0.1178 |
| t_std | 255 | 2000 | 0.7843 | 0.2157 | 0.0 | 12.426 | 0.137 | 0.1065 |
| t_std | 299 | 2044 | 0.7832 | 0.2168 | 0.0 | 12.308 | 0.15 | 0.1055 |

## 5. 시나리오

로봇 충돌 rand 대비 차이는 다른 에피소드라 짝이 아니며, SE는 두 비율의 독립 SE다.

(해당하는 끝난 실행 없음)

## 6. 통로 상자 틈별 통과

통로 안 통과 %는 전역 경로 위에 통로 상자가 있는 에피소드(경로 위 통로 상자 에피소드) 중 성공 비율이며, 궤적이 통로 안에 머물렀는지 확인한 값이 아니다. SE는 이항 SE sqrt(p(1-p)/n). 틈은 경로 위 통로 상자 옆 지나갈 폭 중 가장 작은 값이고 칸 경계는 box_gap.EDGES. 에피소드가 없는 칸의 성공 %는 빈칸.

| 정책 | 칸 | 경로 위 통로 상자 에피소드 | 통로 안 통과 % | 그 SE %p | 틈 < 0.13 m 수 | 틈 < 0.13 m 성공 % | 틈 0.13-0.16 m 수 | 틈 0.13-0.16 m 성공 % | 틈 0.16-0.19 m 수 | 틈 0.16-0.19 m 성공 % | 틈 0.19-0.22 m 수 | 틈 0.19-0.22 m 성공 % | 틈 0.22-0.25 m 수 | 틈 0.22-0.25 m 성공 % | 틈 >= 0.25 m 수 | 틈 >= 0.25 m 성공 % |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| acd3 | aisle | 2792 | 36.35 | 0.9103 | 0 |  | 691 | 3.04 | 757 | 20.48 | 751 | 46.07 | 593 | 83.14 | 0 |  |
| acd3 | aislet | 2792 | 36.39 | 0.9105 | 0 |  | 691 | 2.89 | 757 | 20.48 | 751 | 46.21 | 593 | 83.31 | 0 |  |
| acd3 | demo1 | 2792 | 23.03 | 0.7968 | 0 |  | 691 | 1.88 | 757 | 14.66 | 751 | 31.96 | 593 | 47.05 | 0 |  |
| acd3 | demo1c08 | 2792 | 24.0 | 0.8083 | 0 |  | 691 | 2.03 | 757 | 15.98 | 751 | 34.22 | 593 | 46.88 | 0 |  |
| acd3 | demo1t | 2792 | 23.1 | 0.7976 | 0 |  | 691 | 1.88 | 757 | 14.66 | 751 | 32.49 | 593 | 46.71 | 0 |  |
| acd3 | demo2 | 2792 | 17.69 | 0.7222 | 0 |  | 664 | 1.81 | 752 | 11.44 | 801 | 23.22 | 575 | 36.52 | 0 |  |
| ctrl | aisle | 2792 | 34.74 | 0.9011 | 0 |  | 691 | 2.03 | 757 | 14.93 | 751 | 45.81 | 593 | 84.15 | 0 |  |
| ctrl | demo1 | 2792 | 22.13 | 0.7856 | 0 |  | 691 | 0.72 | 757 | 12.42 | 751 | 30.23 | 593 | 49.24 | 0 |  |
| ctrl | demo2 | 2792 | 16.87 | 0.7087 | 0 |  | 664 | 1.66 | 752 | 10.37 | 801 | 22.1 | 575 | 35.65 | 0 |  |
| demo | aisle | 2792 | 35.42 | 0.9051 | 0 |  | 691 | 3.18 | 757 | 18.49 | 751 | 44.87 | 593 | 82.63 | 0 |  |
| demo | demo1 | 2792 | 20.27 | 0.7608 | 0 |  | 691 | 1.74 | 757 | 12.55 | 751 | 26.76 | 593 | 43.51 | 0 |  |
| demo | demo2 | 2792 | 16.98 | 0.7106 | 0 |  | 664 | 1.81 | 752 | 10.77 | 801 | 21.97 | 575 | 35.65 | 0 |  |
| demo | head | 49 | 18.37 | 5.532 | 0 |  | 5 | 0.0 | 21 | 4.76 | 12 | 50.0 | 11 | 18.18 | 0 |  |
| demo | rand | 49 | 46.94 | 7.1295 | 0 |  | 5 | 0.0 | 21 | 38.1 | 12 | 66.67 | 11 | 63.64 | 0 |  |
| demohist | aisle | 2792 | 59.2 | 0.9301 | 0 |  | 691 | 4.92 | 757 | 52.05 | 751 | 86.28 | 593 | 97.3 | 0 |  |
| demohist | aislel958 | 2792 | 60.78 | 0.924 | 0 |  | 691 | 6.66 | 757 | 54.16 | 751 | 88.02 | 593 | 97.81 | 0 |  |
| demohist | aislepost | 2792 | 53.01 | 0.9445 | 0 |  | 691 | 4.49 | 757 | 44.39 | 751 | 74.83 | 593 | 92.92 | 0 |  |
| demohist | aislepostm | 2792 | 23.39 | 0.8011 | 0 |  | 691 | 18.09 | 757 | 22.32 | 751 | 23.83 | 593 | 30.35 | 0 |  |
| demohist | aislepostmf | 2792 | 58.63 | 0.9321 | 0 |  | 691 | 5.21 | 757 | 50.99 | 751 | 85.09 | 593 | 97.13 | 0 |  |
| demohist | aislepostn | 2792 | 58.7 | 0.9318 | 0 |  | 691 | 5.07 | 757 | 50.07 | 751 | 85.89 | 593 | 97.81 | 0 |  |
| demohist | aislet | 2792 | 59.24 | 0.93 | 0 |  | 691 | 4.78 | 757 | 52.31 | 751 | 86.28 | 593 | 97.3 | 0 |  |
| demohist | demo1 | 2792 | 36.57 | 0.9115 | 0 |  | 691 | 3.91 | 757 | 33.55 | 751 | 53.13 | 593 | 57.5 | 0 |  |
| demohist | demo1c08 | 2792 | 35.89 | 0.9078 | 0 |  | 691 | 3.33 | 757 | 32.36 | 751 | 53.79 | 593 | 55.65 | 0 |  |
| demohist | demo1l958 | 2792 | 36.86 | 0.913 | 0 |  | 691 | 4.2 | 757 | 33.55 | 751 | 54.19 | 593 | 57.17 | 0 |  |
| demohist | demo1maskw | 2792 | 5.59 | 0.4348 | 0 |  | 691 | 4.2 | 757 | 5.02 | 751 | 5.73 | 593 | 7.76 | 0 |  |
| demohist | demo1post | 2792 | 30.87 | 0.8743 | 0 |  | 691 | 3.47 | 757 | 26.95 | 751 | 43.54 | 593 | 51.77 | 0 |  |
| demohist | demo1postm | 2792 | 5.84 | 0.4438 | 0 |  | 691 | 4.2 | 757 | 5.42 | 751 | 6.39 | 593 | 7.59 | 0 |  |
| demohist | demo1postmf | 2792 | 35.1 | 0.9033 | 0 |  | 691 | 3.47 | 757 | 32.23 | 751 | 49.93 | 593 | 56.83 | 0 |  |
| demohist | demo1postn | 2792 | 35.78 | 0.9072 | 0 |  | 691 | 3.91 | 757 | 32.5 | 751 | 52.06 | 593 | 56.49 | 0 |  |
| demohist | demo1t | 2792 | 36.25 | 0.9098 | 0 |  | 691 | 3.33 | 757 | 32.89 | 751 | 53.79 | 593 | 56.66 | 0 |  |
| demohist | demo1tc26a60 | 2792 | 36.57 | 0.9115 | 0 |  | 691 | 2.89 | 757 | 33.03 | 751 | 54.06 | 593 | 58.18 | 0 |  |
| demohist | demo1tc26a90 | 2792 | 35.17 | 0.9037 | 0 |  | 691 | 1.88 | 757 | 31.57 | 751 | 53.0 | 593 | 55.99 | 0 |  |
| demohist | demo1tf15a60 | 2792 | 0.32 | 0.1069 | 0 |  | 691 | 0.29 | 757 | 0.4 | 751 | 0.27 | 593 | 0.34 | 0 |  |
| demohist | demo1tf20a30 | 2792 | 13.79 | 0.6525 | 0 |  | 691 | 2.03 | 757 | 1.06 | 751 | 15.98 | 593 | 40.98 | 0 |  |
| demohist | demo1tf20a60 | 2792 | 0.0 | 0.0 | 0 |  | 691 | 0.0 | 757 | 0.0 | 751 | 0.0 | 593 | 0.0 | 0 |  |
| demohist | demo1tp30a60 | 2792 | 41.69 | 0.9331 | 0 |  | 691 | 3.91 | 757 | 36.99 | 751 | 62.32 | 593 | 65.6 | 0 |  |
| demohist | demo1tp30a90 | 2792 | 40.94 | 0.9306 | 0 |  | 691 | 4.2 | 757 | 36.06 | 751 | 60.45 | 593 | 65.26 | 0 |  |
| demohist | demo1tw26a60 | 2792 | 42.87 | 0.9366 | 0 |  | 691 | 3.91 | 757 | 38.18 | 751 | 64.71 | 593 | 66.61 | 0 |  |
| demohist | demo1tw26a60n | 2792 | 42.3 | 0.935 | 0 |  | 691 | 4.2 | 757 | 37.65 | 751 | 62.98 | 593 | 66.44 | 0 |  |
| demohist | demo2 | 2792 | 30.16 | 0.8686 | 0 |  | 664 | 4.52 | 752 | 28.99 | 801 | 42.32 | 575 | 44.35 | 0 |  |
| demohist | head | 49 | 30.61 | 6.5839 | 0 |  | 5 | 0.0 | 21 | 28.57 | 12 | 58.33 | 11 | 18.18 | 0 |  |
| demohist | rand | 49 | 69.39 | 6.5839 | 0 |  | 5 | 20.0 | 21 | 76.19 | 12 | 75.0 | 11 | 72.73 | 0 |  |
| demohist2 | aisle | 2792 | 61.93 | 0.9189 | 0 |  | 691 | 6.08 | 757 | 57.07 | 751 | 90.55 | 593 | 96.96 | 0 |  |
| demohist2 | demo1 | 2792 | 39.04 | 0.9233 | 0 |  | 691 | 4.49 | 757 | 38.18 | 751 | 56.86 | 593 | 57.84 | 0 |  |
| demohist2 | demo2 | 2792 | 32.7 | 0.8878 | 0 |  | 664 | 5.27 | 752 | 34.57 | 801 | 44.44 | 575 | 45.57 | 0 |  |
| demohist2 | head | 49 | 34.69 | 6.7998 | 0 |  | 5 | 0.0 | 21 | 38.1 | 12 | 58.33 | 11 | 18.18 | 0 |  |
| demohist2 | rand | 49 | 69.39 | 6.5839 | 0 |  | 5 | 20.0 | 21 | 80.95 | 12 | 75.0 | 11 | 63.64 | 0 |  |
| demohist2m | demo1 | 2792 | 39.4 | 0.9248 | 0 |  | 691 | 3.62 | 757 | 38.97 | 751 | 57.39 | 593 | 58.85 | 0 |  |
| demohistdyn | aisle | 2792 | 53.37 | 0.9441 | 0 |  | 691 | 3.18 | 757 | 34.74 | 751 | 83.62 | 593 | 97.3 | 0 |  |
| demohistdyn | demo1 | 2792 | 31.48 | 0.879 | 0 |  | 691 | 2.17 | 757 | 25.1 | 751 | 47.8 | 593 | 53.12 | 0 |  |
| demohistdyn | demo2 | 2792 | 26.15 | 0.8317 | 0 |  | 664 | 3.16 | 752 | 20.08 | 801 | 37.83 | 575 | 44.35 | 0 |  |
| demohistdyn | rand | 49 | 67.35 | 6.699 | 0 |  | 5 | 0.0 | 21 | 66.67 | 12 | 83.33 | 11 | 81.82 | 0 |  |
| demostd | aisle | 2792 | 56.59 | 0.938 | 0 |  | 691 | 4.2 | 757 | 41.74 | 751 | 86.15 | 593 | 99.16 | 0 |  |
| demostd | demo1 | 2792 | 34.63 | 0.9004 | 0 |  | 691 | 3.04 | 757 | 28.8 | 751 | 51.26 | 593 | 57.84 | 0 |  |
| demostd | demo2 | 2792 | 29.51 | 0.8632 | 0 |  | 664 | 3.92 | 752 | 26.06 | 801 | 41.45 | 575 | 46.96 | 0 |  |
| demostd | head | 49 | 38.78 | 6.9607 | 0 |  | 5 | 0.0 | 21 | 38.1 | 12 | 58.33 | 11 | 36.36 | 0 |  |
| demostd | rand | 49 | 73.47 | 6.307 | 0 |  | 5 | 20.0 | 21 | 76.19 | 12 | 83.33 | 11 | 81.82 | 0 |  |
| demostd2 | aisle | 2792 | 60.28 | 0.926 | 0 |  | 691 | 5.21 | 757 | 52.71 | 751 | 88.68 | 593 | 98.15 | 0 |  |
| demostd2 | demo1 | 2792 | 37.39 | 0.9157 | 0 |  | 691 | 3.18 | 757 | 34.08 | 751 | 55.26 | 593 | 58.85 | 0 |  |
| demostd2 | demo2 | 2792 | 31.12 | 0.8762 | 0 |  | 664 | 4.52 | 752 | 31.38 | 801 | 42.95 | 575 | 45.04 | 0 |  |
| demostd2 | rand | 49 | 69.39 | 6.5839 | 0 |  | 5 | 0.0 | 21 | 76.19 | 12 | 75.0 | 11 | 81.82 | 0 |  |
| demostd2m | demo1 | 2792 | 35.39 | 0.905 | 0 |  | 691 | 3.47 | 757 | 32.36 | 751 | 51.0 | 593 | 56.66 | 0 |  |
| histstd | aisle | 2792 | 54.08 | 0.9431 | 0 |  | 691 | 3.76 | 757 | 36.99 | 751 | 83.22 | 593 | 97.64 | 0 |  |
| histstd | demo1 | 2792 | 34.28 | 0.8983 | 0 |  | 691 | 2.89 | 757 | 27.48 | 751 | 52.46 | 593 | 56.49 | 0 |  |
| histstd | demo2 | 2792 | 27.94 | 0.8492 | 0 |  | 664 | 3.16 | 752 | 23.14 | 801 | 40.45 | 575 | 45.39 | 0 |  |
| histstd | head | 49 | 34.69 | 6.7998 | 0 |  | 5 | 0.0 | 21 | 38.1 | 12 | 50.0 | 11 | 27.27 | 0 |  |
| histstd | rand | 49 | 67.35 | 6.699 | 0 |  | 5 | 0.0 | 21 | 66.67 | 12 | 91.67 | 11 | 72.73 | 0 |  |
| std | aisle | 2792 | 52.97 | 0.9446 | 0 |  | 691 | 2.6 | 757 | 38.71 | 751 | 79.23 | 593 | 96.63 | 0 |  |
| std | aislet | 2792 | 53.04 | 0.9445 | 0 |  | 691 | 2.6 | 757 | 38.57 | 751 | 79.63 | 593 | 96.63 | 0 |  |
| std | demo1 | 2792 | 35.92 | 0.908 | 0 |  | 691 | 2.46 | 757 | 32.36 | 751 | 53.0 | 593 | 57.84 | 0 |  |
| std | demo1c08 | 2792 | 37.54 | 0.9164 | 0 |  | 691 | 3.62 | 757 | 35.8 | 751 | 54.73 | 593 | 57.5 | 0 |  |
| std | demo1t | 2792 | 36.07 | 0.9088 | 0 |  | 691 | 2.46 | 757 | 31.84 | 751 | 53.0 | 593 | 59.19 | 0 |  |
| std | demo2 | 2792 | 29.76 | 0.8653 | 0 |  | 664 | 3.01 | 752 | 28.86 | 801 | 41.45 | 575 | 45.57 | 0 |  |

## 6b. 통로 상자 에피소드 짝 비교 (a - 대조)

aisle·demo1·demo2 비교에서 두 실행 모두 box_gap_m이 유한한 에피소드만 짝지은 성공·충돌 차이(%p)와 SE. 틈 칸은 a 실행의 틈으로 나눈다. 상자 에피소드 일치가 same이 아니면 두 실행에서 경로 위 통로 상자 에피소드(또는 그 틈)가 달라, 양쪽 모두 유한한 에피소드만 쓴 값이다. 2SE는 2절과 같다. 에피소드가 없는 칸은 빈칸, 1개면 SE가 빈칸.

| 정책 | 칸 | 대조 정책 | 대조 칸 | 같은 에피소드 | 상자 에피소드 일치 | 틈 칸 m | 짝 수 | 성공 차 %p | 성공 SE | 성공 2SE | 충돌 차 %p | 충돌 SE | 충돌 2SE |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| demo | aisle | ctrl | aisle | yes (16 tracked envs) | same | all | 2792 | 0.6805 | 0.6614 | inside | 6.7693 | 0.8249 | outside |
| demo | aisle | ctrl | aisle | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demo | aisle | ctrl | aisle | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 1.1577 | 0.6128 | inside | 15.6295 | 1.7921 | outside |
| demo | aisle | ctrl | aisle | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 3.5667 | 1.1527 | outside | 4.3593 | 1.5948 | outside |
| demo | aisle | ctrl | aisle | yes (16 tracked envs) | same | 0.19-0.22 | 751 | -0.9321 | 1.5706 | inside | 3.4621 | 1.6487 | outside |
| demo | aisle | ctrl | aisle | yes (16 tracked envs) | same | 0.22-0.25 | 593 | -1.5177 | 1.7447 | inside | 3.7099 | 1.424 | outside |
| demo | aisle | ctrl | aisle | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demo | aisle | acd3 | aisle | yes (16 tracked envs) | same | all | 2792 | -0.9312 | 0.5992 | inside | 6.2679 | 0.7549 | outside |
| demo | aisle | acd3 | aisle | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demo | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 0.1447 | 0.5971 | inside | 10.9986 | 1.5574 | outside |
| demo | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.16-0.19 | 757 | -1.9815 | 1.1577 | inside | 10.3038 | 1.53 | outside |
| demo | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.19-0.22 | 751 | -1.1984 | 1.465 | inside | 2.6631 | 1.5622 | inside |
| demo | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.22-0.25 | 593 | -0.5059 | 1.3606 | inside | 0.1686 | 1.2053 | inside |
| demo | aisle | acd3 | aisle | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demostd | aisle | std | aisle | yes (16 tracked envs) | same | all | 2792 | 3.6175 | 0.7924 | outside | 8.1662 | 0.8958 | outside |
| demostd | aisle | std | aisle | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demostd | aisle | std | aisle | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 1.5919 | 0.9253 | inside | 25.7598 | 1.9221 | outside |
| demostd | aisle | std | aisle | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 3.0383 | 2.0746 | inside | 9.6433 | 2.0731 | outside |
| demostd | aisle | std | aisle | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 6.9241 | 1.7797 | outside | -1.8642 | 1.6413 | inside |
| demostd | aisle | std | aisle | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 2.5295 | 0.8027 | outside | -1.5177 | 0.6931 | outside |
| demostd | aisle | std | aisle | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demostd | aisle | demo | aisle | yes (16 tracked envs) | same | all | 2792 | 21.1676 | 0.9464 | outside | -17.192 | 0.9846 | outside |
| demostd | aisle | demo | aisle | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demostd | aisle | demo | aisle | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 1.013 | 0.9707 | inside | 1.8813 | 1.5782 | inside |
| demostd | aisle | demo | aisle | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 23.2497 | 2.0694 | outside | -20.6077 | 2.1144 | outside |
| demostd | aisle | demo | aisle | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 41.2783 | 2.1404 | outside | -36.4847 | 2.1404 | outside |
| demostd | aisle | demo | aisle | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 16.5261 | 1.5994 | outside | -10.6239 | 1.3743 | outside |
| demostd | aisle | demo | aisle | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demostd | aisle | acd3 | aisle | yes (16 tracked envs) | same | all | 2792 | 20.2364 | 0.963 | outside | -10.9241 | 1.0381 | outside |
| demostd | aisle | acd3 | aisle | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demostd | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 1.1577 | 0.9812 | inside | 12.8799 | 1.8633 | outside |
| demostd | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 21.2682 | 2.1911 | outside | -10.3038 | 2.2039 | outside |
| demostd | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 40.0799 | 2.1334 | outside | -33.8216 | 2.1156 | outside |
| demostd | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 16.0202 | 1.5992 | outside | -10.4553 | 1.3661 | outside |
| demostd | aisle | acd3 | aisle | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| ctrl | aisle | acd3 | aisle | yes (16 tracked envs) | same | all | 2792 | -1.6117 | 0.6491 | outside | -0.5014 | 0.7994 | inside |
| ctrl | aisle | acd3 | aisle | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| ctrl | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.13-0.16 | 691 | -1.013 | 0.5596 | inside | -4.631 | 1.7768 | outside |
| ctrl | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.16-0.19 | 757 | -5.5482 | 1.2235 | outside | 5.9445 | 1.588 | outside |
| ctrl | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.19-0.22 | 751 | -0.2663 | 1.4956 | inside | -0.7989 | 1.4954 | inside |
| ctrl | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 1.0118 | 1.6873 | inside | -3.5413 | 1.4347 | outside |
| ctrl | aisle | acd3 | aisle | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| std | aisle | acd3 | aisle | yes (16 tracked envs) | same | all | 2792 | 16.6189 | 0.947 | outside | -19.0903 | 1.0742 | outside |
| std | aisle | acd3 | aisle | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| std | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.13-0.16 | 691 | -0.4342 | 0.7239 | inside | -12.8799 | 2.306 | outside |
| std | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 18.2299 | 2.1835 | outside | -19.9472 | 2.3232 | outside |
| std | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 33.1558 | 2.1668 | outside | -31.9574 | 2.0954 | outside |
| std | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 13.4907 | 1.6468 | outside | -8.9376 | 1.3945 | outside |
| std | aisle | acd3 | aisle | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| std | aisle | ctrl | aisle | yes (16 tracked envs) | same | all | 2792 | 18.2307 | 0.9398 | outside | -18.5888 | 1.0642 | outside |
| std | aisle | ctrl | aisle | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| std | aisle | ctrl | aisle | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 0.5789 | 0.6473 | inside | -8.2489 | 2.2262 | outside |
| std | aisle | ctrl | aisle | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 23.7781 | 2.1608 | outside | -25.8917 | 2.3619 | outside |
| std | aisle | ctrl | aisle | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 33.4221 | 2.1858 | outside | -31.1585 | 2.086 | outside |
| std | aisle | ctrl | aisle | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 12.4789 | 1.5901 | outside | -5.3963 | 1.1726 | outside |
| std | aisle | ctrl | aisle | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demo | demo1 | ctrl | demo1 | yes (16 tracked envs) | same | all | 2792 | -1.8625 | 0.6498 | outside | 5.1576 | 0.7399 | outside |
| demo | demo1 | ctrl | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demo | demo1 | ctrl | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 1.013 | 0.4328 | outside | 5.7887 | 1.2599 | outside |
| demo | demo1 | ctrl | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 0.1321 | 1.1448 | inside | 2.2457 | 1.2722 | inside |
| demo | demo1 | ctrl | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | -3.4621 | 1.5939 | outside | 5.5925 | 1.63 | outside |
| demo | demo1 | ctrl | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | -5.7336 | 1.6882 | outside | 7.5885 | 1.7506 | outside |
| demo | demo1 | ctrl | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demo | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | all | 2792 | -2.7579 | 0.6172 | outside | 6.5903 | 0.7288 | outside |
| demo | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demo | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | -0.1447 | 0.5609 | inside | 6.657 | 1.2368 | outside |
| demo | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | -2.1136 | 1.1798 | inside | 6.605 | 1.3654 | outside |
| demo | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | -5.1931 | 1.4534 | outside | 7.3236 | 1.548 | outside |
| demo | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | -3.5413 | 1.5307 | outside | 5.5649 | 1.6807 | outside |
| demo | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demostd | demo1 | std | demo1 | yes (16 tracked envs) | same | all | 2792 | -1.2894 | 0.807 | inside | 5.9097 | 0.8914 | outside |
| demostd | demo1 | std | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demostd | demo1 | std | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 0.5789 | 0.7929 | inside | 9.8408 | 1.5141 | outside |
| demostd | demo1 | std | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | -3.5667 | 1.9429 | inside | 8.1902 | 1.9913 | outside |
| demostd | demo1 | std | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | -1.731 | 1.8014 | inside | 3.4621 | 1.842 | inside |
| demostd | demo1 | std | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 0.0 | 1.4906 | inside | 1.5177 | 1.591 | inside |
| demostd | demo1 | std | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demostd | demo1 | demo | demo1 | yes (16 tracked envs) | same | all | 2792 | 14.3625 | 0.8697 | outside | -13.9685 | 0.8916 | outside |
| demostd | demo1 | demo | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demostd | demo1 | demo | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 1.3025 | 0.7509 | inside | -1.013 | 1.0335 | inside |
| demostd | demo1 | demo | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 16.2483 | 1.8648 | outside | -15.7199 | 1.8708 | outside |
| demostd | demo1 | demo | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 24.5007 | 2.0248 | outside | -24.2344 | 2.0378 | outside |
| demostd | demo1 | demo | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 14.3339 | 1.7927 | outside | -13.828 | 1.8073 | outside |
| demostd | demo1 | demo | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demostd | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | all | 2792 | 11.6046 | 0.9192 | outside | -7.3782 | 0.9933 | outside |
| demostd | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demostd | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 1.1577 | 0.792 | inside | 5.644 | 1.3647 | outside |
| demostd | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 14.1347 | 1.9422 | outside | -9.1149 | 2.0509 | outside |
| demostd | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 19.3076 | 2.1854 | outside | -16.9108 | 2.2117 | outside |
| demostd | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 10.7926 | 1.9324 | outside | -8.2631 | 2.032 | outside |
| demostd | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| ctrl | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | all | 2792 | -0.8954 | 0.6003 | inside | 1.4327 | 0.7159 | outside |
| ctrl | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| ctrl | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | -1.1577 | 0.4558 | outside | 0.8683 | 1.2786 | inside |
| ctrl | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | -2.2457 | 1.1869 | inside | 4.3593 | 1.3322 | outside |
| ctrl | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | -1.731 | 1.3639 | inside | 1.731 | 1.4522 | inside |
| ctrl | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 2.1922 | 1.5534 | inside | -2.0236 | 1.6857 | inside |
| ctrl | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| std | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | all | 2792 | 12.894 | 0.9031 | outside | -13.288 | 1.0156 | outside |
| std | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| std | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 0.5789 | 0.7381 | inside | -4.1968 | 1.5982 | outside |
| std | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 17.7015 | 1.9796 | outside | -17.3052 | 2.141 | outside |
| std | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 21.0386 | 2.1184 | outside | -20.3728 | 2.2049 | outside |
| std | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 10.7926 | 1.8107 | outside | -9.7808 | 1.9561 | outside |
| std | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| std | demo1 | ctrl | demo1 | yes (16 tracked envs) | same | all | 2792 | 13.7894 | 0.9019 | outside | -14.7206 | 0.9969 | outside |
| std | demo1 | ctrl | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| std | demo1 | ctrl | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 1.7366 | 0.6761 | outside | -5.0651 | 1.6076 | outside |
| std | demo1 | ctrl | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 19.9472 | 1.9287 | outside | -21.6645 | 2.0668 | outside |
| std | demo1 | ctrl | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 22.7696 | 2.1405 | outside | -22.1039 | 2.1703 | outside |
| std | demo1 | ctrl | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 8.6003 | 1.8536 | outside | -7.7572 | 1.8827 | outside |
| std | demo1 | ctrl | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demo | demo2 | ctrl | demo2 | yes (16 tracked envs) | same | all | 2792 | 0.1074 | 0.5561 | inside | 3.9398 | 0.7126 | outside |
| demo | demo2 | ctrl | demo2 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demo | demo2 | ctrl | demo2 | yes (16 tracked envs) | same | 0.13-0.16 | 664 | 0.1506 | 0.6214 | inside | 5.4217 | 1.2249 | outside |
| demo | demo2 | ctrl | demo2 | yes (16 tracked envs) | same | 0.16-0.19 | 752 | 0.3989 | 1.1052 | inside | 3.1915 | 1.416 | outside |
| demo | demo2 | ctrl | demo2 | yes (16 tracked envs) | same | 0.19-0.22 | 801 | -0.1248 | 1.2047 | inside | 4.1199 | 1.4549 | outside |
| demo | demo2 | ctrl | demo2 | yes (16 tracked envs) | same | 0.22-0.25 | 575 | 0.0 | 1.3706 | inside | 2.9565 | 1.5617 | inside |
| demo | demo2 | ctrl | demo2 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demo | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | all | 2792 | -0.7163 | 0.5525 | inside | 6.841 | 0.7306 | outside |
| demo | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demo | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.13-0.16 | 664 | 0.0 | 0.6394 | inside | 9.1867 | 1.3928 | outside |
| demo | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.16-0.19 | 752 | -0.6649 | 1.0725 | inside | 6.117 | 1.3904 | outside |
| demo | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.19-0.22 | 801 | -1.2484 | 1.171 | inside | 7.1161 | 1.4404 | outside |
| demo | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.22-0.25 | 575 | -0.8696 | 1.4243 | inside | 4.6957 | 1.6304 | outside |
| demo | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demostd | demo2 | std | demo2 | yes (16 tracked envs) | same | all | 2792 | -0.2507 | 0.7506 | inside | 5.4441 | 0.8581 | outside |
| demostd | demo2 | std | demo2 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demostd | demo2 | std | demo2 | yes (16 tracked envs) | same | 0.13-0.16 | 664 | 0.9036 | 0.8781 | inside | 7.9819 | 1.5427 | outside |
| demostd | demo2 | std | demo2 | yes (16 tracked envs) | same | 0.16-0.19 | 752 | -2.7926 | 1.8168 | inside | 6.9149 | 1.8931 | outside |
| demostd | demo2 | std | demo2 | yes (16 tracked envs) | same | 0.19-0.22 | 801 | 0.0 | 1.4781 | inside | 3.8702 | 1.6278 | outside |
| demostd | demo2 | std | demo2 | yes (16 tracked envs) | same | 0.22-0.25 | 575 | 1.3913 | 1.5362 | inside | 2.7826 | 1.7015 | inside |
| demostd | demo2 | std | demo2 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demostd | demo2 | demo | demo2 | yes (16 tracked envs) | same | all | 2792 | 12.5358 | 0.7979 | outside | -11.4971 | 0.8538 | outside |
| demostd | demo2 | demo | demo2 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demostd | demo2 | demo | demo2 | yes (16 tracked envs) | same | 0.13-0.16 | 664 | 2.1084 | 0.7641 | outside | -2.7108 | 1.1627 | outside |
| demostd | demo2 | demo | demo2 | yes (16 tracked envs) | same | 0.16-0.19 | 752 | 15.2926 | 1.6801 | outside | -13.6968 | 1.7698 | outside |
| demostd | demo2 | demo | demo2 | yes (16 tracked envs) | same | 0.19-0.22 | 801 | 19.4757 | 1.7382 | outside | -18.2272 | 1.8077 | outside |
| demostd | demo2 | demo | demo2 | yes (16 tracked envs) | same | 0.22-0.25 | 575 | 11.3043 | 1.7892 | outside | -9.3913 | 1.8497 | outside |
| demostd | demo2 | demo | demo2 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demostd | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | all | 2792 | 11.8195 | 0.8034 | outside | -4.6562 | 0.9532 | outside |
| demostd | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demostd | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.13-0.16 | 664 | 2.1084 | 0.7641 | outside | 6.4759 | 1.5088 | outside |
| demostd | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.16-0.19 | 752 | 14.6277 | 1.6827 | outside | -7.5798 | 1.9588 | outside |
| demostd | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.19-0.22 | 801 | 18.2272 | 1.7816 | outside | -11.1111 | 1.9953 | outside |
| demostd | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.22-0.25 | 575 | 10.4348 | 1.7729 | outside | -4.6957 | 1.9362 | outside |
| demostd | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| ctrl | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | all | 2792 | -0.8238 | 0.4974 | inside | 2.9011 | 0.7008 | outside |
| ctrl | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| ctrl | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.13-0.16 | 664 | -0.1506 | 0.4521 | inside | 3.7651 | 1.3485 | outside |
| ctrl | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.16-0.19 | 752 | -1.0638 | 0.995 | inside | 2.9255 | 1.3658 | outside |
| ctrl | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.19-0.22 | 801 | -1.1236 | 1.0811 | inside | 2.9963 | 1.3529 | outside |
| ctrl | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.22-0.25 | 575 | -0.8696 | 1.2667 | inside | 1.7391 | 1.5552 | inside |
| ctrl | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| std | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | all | 2792 | 12.0702 | 0.814 | outside | -10.1003 | 0.9924 | outside |
| std | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| std | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.13-0.16 | 664 | 1.2048 | 0.8242 | inside | -1.506 | 1.8326 | inside |
| std | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.16-0.19 | 752 | 17.4202 | 1.7662 | outside | -14.4947 | 2.0321 | outside |
| std | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.19-0.22 | 801 | 18.2272 | 1.764 | outside | -14.9813 | 1.9832 | outside |
| std | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.22-0.25 | 575 | 9.0435 | 1.717 | outside | -7.4783 | 1.9522 | outside |
| std | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| std | demo2 | ctrl | demo2 | yes (16 tracked envs) | same | all | 2792 | 12.894 | 0.8134 | outside | -13.0014 | 0.9557 | outside |
| std | demo2 | ctrl | demo2 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| std | demo2 | ctrl | demo2 | yes (16 tracked envs) | same | 0.13-0.16 | 664 | 1.3554 | 0.8375 | inside | -5.2711 | 1.726 | outside |
| std | demo2 | ctrl | demo2 | yes (16 tracked envs) | same | 0.16-0.19 | 752 | 18.484 | 1.7818 | outside | -17.4202 | 2.0276 | outside |
| std | demo2 | ctrl | demo2 | yes (16 tracked envs) | same | 0.19-0.22 | 801 | 19.3508 | 1.7445 | outside | -17.9775 | 1.8701 | outside |
| std | demo2 | ctrl | demo2 | yes (16 tracked envs) | same | 0.22-0.25 | 575 | 9.913 | 1.6997 | outside | -9.2174 | 1.8757 | outside |
| std | demo2 | ctrl | demo2 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohist | demo1 | demostd | demo1 | yes (16 tracked envs) | same | all | 2792 | 1.9341 | 0.7488 | outside | -2.9728 | 0.7853 | outside |
| demohist | demo1 | demostd | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohist | demo1 | demostd | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 0.8683 | 0.7925 | inside | -5.2098 | 1.1953 | outside |
| demohist | demo1 | demostd | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 4.7556 | 1.7945 | outside | -4.8877 | 1.7989 | outside |
| demohist | demo1 | demostd | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 1.8642 | 1.6628 | inside | -1.5979 | 1.6738 | inside |
| demohist | demo1 | demostd | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | -0.3373 | 1.3711 | inside | 0.3373 | 1.3917 | inside |
| demohist | demo1 | demostd | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohist | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | all | 2792 | 13.5387 | 0.9375 | outside | -10.351 | 1.0252 | outside |
| demohist | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohist | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 2.026 | 0.8894 | outside | 0.4342 | 1.553 | inside |
| demohist | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 18.8904 | 2.0478 | outside | -14.0026 | 2.1767 | outside |
| demohist | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 21.1718 | 2.1124 | outside | -18.5087 | 2.162 | outside |
| demohist | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 10.4553 | 1.9935 | outside | -7.9258 | 2.1031 | outside |
| demohist | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohist | demo1 | std | demo1 | yes (16 tracked envs) | same | all | 2792 | 0.6447 | 0.8199 | inside | 2.937 | 0.9172 | outside |
| demohist | demo1 | std | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohist | demo1 | std | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 1.4472 | 0.7913 | inside | 4.631 | 1.6545 | outside |
| demohist | demo1 | std | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 1.1889 | 2.0259 | inside | 3.3025 | 2.0908 | inside |
| demohist | demo1 | std | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 0.1332 | 1.7424 | inside | 1.8642 | 1.7764 | inside |
| demohist | demo1 | std | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | -0.3373 | 1.5832 | inside | 1.855 | 1.6605 | inside |
| demohist | demo1 | std | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohist | demo2 | demostd | demo2 | yes (16 tracked envs) | same | all | 2792 | 0.6447 | 0.6834 | inside | -1.1461 | 0.7443 | inside |
| demohist | demo2 | demostd | demo2 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohist | demo2 | demostd | demo2 | yes (16 tracked envs) | same | 0.13-0.16 | 664 | 0.6024 | 0.9288 | inside | -2.5602 | 1.386 | inside |
| demohist | demo2 | demostd | demo2 | yes (16 tracked envs) | same | 0.16-0.19 | 752 | 2.9255 | 1.6043 | inside | -2.6596 | 1.6268 | inside |
| demohist | demo2 | demostd | demo2 | yes (16 tracked envs) | same | 0.19-0.22 | 801 | 0.8739 | 1.3393 | inside | -0.6242 | 1.3965 | inside |
| demohist | demo2 | demostd | demo2 | yes (16 tracked envs) | same | 0.22-0.25 | 575 | -2.6087 | 1.3991 | inside | 1.7391 | 1.4752 | inside |
| demohist | demo2 | demostd | demo2 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohist | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | all | 2792 | 12.4642 | 0.8406 | outside | -5.8023 | 0.9931 | outside |
| demohist | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohist | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.13-0.16 | 664 | 2.7108 | 0.8981 | outside | 3.9157 | 1.6714 | outside |
| demohist | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.16-0.19 | 752 | 17.5532 | 1.8188 | outside | -10.2394 | 2.0404 | outside |
| demohist | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.19-0.22 | 801 | 19.1011 | 1.835 | outside | -11.7353 | 2.0564 | outside |
| demohist | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.22-0.25 | 575 | 7.8261 | 1.7361 | outside | -2.9565 | 1.9731 | inside |
| demohist | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohist | demo2 | std | demo2 | yes (16 tracked envs) | same | all | 2792 | 0.394 | 0.759 | inside | 4.298 | 0.881 | outside |
| demohist | demo2 | std | demo2 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohist | demo2 | std | demo2 | yes (16 tracked envs) | same | 0.13-0.16 | 664 | 1.506 | 0.8506 | inside | 5.4217 | 1.6375 | outside |
| demohist | demo2 | std | demo2 | yes (16 tracked envs) | same | 0.16-0.19 | 752 | 0.133 | 1.8486 | inside | 4.2553 | 1.9128 | outside |
| demohist | demo2 | std | demo2 | yes (16 tracked envs) | same | 0.19-0.22 | 801 | 0.8739 | 1.5143 | inside | 3.2459 | 1.6627 | inside |
| demohist | demo2 | std | demo2 | yes (16 tracked envs) | same | 0.22-0.25 | 575 | -1.2174 | 1.5266 | inside | 4.5217 | 1.7651 | outside |
| demohist | demo2 | std | demo2 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohist | aisle | demostd | aisle | yes (16 tracked envs) | same | all | 2792 | 2.6146 | 0.7048 | outside | -4.6203 | 0.7742 | outside |
| demohist | aisle | demostd | aisle | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohist | aisle | demostd | aisle | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 0.7236 | 0.8805 | inside | -9.8408 | 1.5686 | outside |
| demohist | aisle | demostd | aisle | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 10.3038 | 1.9782 | outside | -10.0396 | 1.9888 | outside |
| demohist | aisle | demostd | aisle | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 0.1332 | 1.3911 | inside | 0.5326 | 1.3587 | inside |
| demohist | aisle | demostd | aisle | yes (16 tracked envs) | same | 0.22-0.25 | 593 | -1.855 | 0.5546 | outside | 1.855 | 0.5546 | outside |
| demohist | aisle | demostd | aisle | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohist | aisle | acd3 | aisle | yes (16 tracked envs) | same | all | 2792 | 22.851 | 1.0159 | outside | -15.5444 | 1.1021 | outside |
| demohist | aisle | acd3 | aisle | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohist | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 1.8813 | 1.0318 | inside | 3.0391 | 2.1005 | inside |
| demohist | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 31.572 | 2.4069 | outside | -20.3435 | 2.4612 | outside |
| demohist | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 40.213 | 2.1917 | outside | -33.2889 | 2.1599 | outside |
| demohist | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 14.1653 | 1.5842 | outside | -8.6003 | 1.4579 | outside |
| demohist | aisle | acd3 | aisle | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohist | aisle | std | aisle | yes (16 tracked envs) | same | all | 2792 | 6.2321 | 0.8287 | outside | 3.5458 | 0.9059 | outside |
| demohist | aisle | std | aisle | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohist | aisle | std | aisle | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 2.3155 | 0.9783 | outside | 15.919 | 2.0506 | outside |
| demohist | aisle | std | aisle | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 13.3421 | 2.2801 | outside | -0.3963 | 2.1841 | inside |
| demohist | aisle | std | aisle | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 7.0573 | 1.6815 | outside | -1.3316 | 1.5067 | inside |
| demohist | aisle | std | aisle | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 0.6745 | 0.8601 | inside | 0.3373 | 0.8267 | inside |
| demohist | aisle | std | aisle | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| histstd | demo1 | std | demo1 | yes (16 tracked envs) | same | all | 2792 | -1.6476 | 0.8131 | outside | 5.1218 | 0.8932 | outside |
| histstd | demo1 | std | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| histstd | demo1 | std | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 0.4342 | 0.8062 | inside | 7.8148 | 1.4753 | outside |
| histstd | demo1 | std | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | -4.8877 | 1.9391 | outside | 8.0581 | 2.0138 | outside |
| histstd | demo1 | std | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | -0.5326 | 1.7676 | inside | 1.5979 | 1.8065 | inside |
| histstd | demo1 | std | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | -1.3491 | 1.6179 | inside | 2.6981 | 1.6841 | inside |
| histstd | demo1 | std | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| histstd | demo1 | demohist | demo1 | yes (16 tracked envs) | same | all | 2792 | -2.2923 | 0.8126 | outside | 2.1848 | 0.8684 | outside |
| histstd | demo1 | demohist | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| histstd | demo1 | demohist | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | -1.013 | 0.9036 | inside | 3.1838 | 1.4431 | outside |
| histstd | demo1 | demohist | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | -6.0766 | 1.9482 | outside | 4.7556 | 1.9885 | outside |
| histstd | demo1 | demohist | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | -0.6658 | 1.7422 | inside | -0.2663 | 1.7576 | inside |
| histstd | demo1 | demohist | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | -1.0118 | 1.5646 | inside | 0.8432 | 1.5739 | inside |
| histstd | demo1 | demohist | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| histstd | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | all | 2792 | 11.2464 | 0.8736 | outside | -8.1662 | 0.9581 | outside |
| histstd | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| histstd | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 1.013 | 0.7231 | inside | 3.6179 | 1.3438 | outside |
| histstd | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 12.8137 | 1.7861 | outside | -9.247 | 1.9496 | outside |
| histstd | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 20.506 | 2.1337 | outside | -18.775 | 2.1671 | outside |
| histstd | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 9.4435 | 1.8233 | outside | -7.0826 | 1.9171 | outside |
| histstd | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| histstd | demo2 | std | demo2 | yes (16 tracked envs) | same | all | 2792 | -1.8266 | 0.7684 | outside | 5.9097 | 0.8755 | outside |
| histstd | demo2 | std | demo2 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| histstd | demo2 | std | demo2 | yes (16 tracked envs) | same | 0.13-0.16 | 664 | 0.1506 | 0.9412 | inside | 6.9277 | 1.6428 | outside |
| histstd | demo2 | std | demo2 | yes (16 tracked envs) | same | 0.16-0.19 | 752 | -5.7181 | 1.778 | outside | 7.7128 | 1.8796 | outside |
| histstd | demo2 | std | demo2 | yes (16 tracked envs) | same | 0.19-0.22 | 801 | -0.9988 | 1.5798 | inside | 4.6192 | 1.6913 | outside |
| histstd | demo2 | std | demo2 | yes (16 tracked envs) | same | 0.22-0.25 | 575 | -0.1739 | 1.5666 | inside | 4.1739 | 1.6966 | outside |
| histstd | demo2 | std | demo2 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| histstd | demo2 | demohist | demo2 | yes (16 tracked envs) | same | all | 2792 | -2.2206 | 0.7417 | outside | 1.6117 | 0.8218 | inside |
| histstd | demo2 | demohist | demo2 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| histstd | demo2 | demohist | demo2 | yes (16 tracked envs) | same | 0.13-0.16 | 664 | -1.3554 | 0.9397 | inside | 1.506 | 1.506 | inside |
| histstd | demo2 | demohist | demo2 | yes (16 tracked envs) | same | 0.16-0.19 | 752 | -5.8511 | 1.8022 | outside | 3.4574 | 1.8776 | inside |
| histstd | demo2 | demohist | demo2 | yes (16 tracked envs) | same | 0.19-0.22 | 801 | -1.8727 | 1.4283 | inside | 1.3733 | 1.5035 | inside |
| histstd | demo2 | demohist | demo2 | yes (16 tracked envs) | same | 0.22-0.25 | 575 | 1.0435 | 1.4967 | inside | -0.3478 | 1.5762 | inside |
| histstd | demo2 | demohist | demo2 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| histstd | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | all | 2792 | 10.2436 | 0.7935 | outside | -4.1905 | 0.9546 | outside |
| histstd | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| histstd | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.13-0.16 | 664 | 1.3554 | 0.8375 | inside | 5.4217 | 1.6513 | outside |
| histstd | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.16-0.19 | 752 | 11.7021 | 1.5615 | outside | -6.7819 | 1.8223 | outside |
| histstd | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.19-0.22 | 801 | 17.2285 | 1.7939 | outside | -10.362 | 2.0236 | outside |
| histstd | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.22-0.25 | 575 | 8.8696 | 1.7961 | outside | -3.3043 | 1.9875 | inside |
| histstd | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| histstd | aisle | std | aisle | yes (16 tracked envs) | same | all | 2792 | 1.1103 | 0.8314 | inside | 6.9484 | 0.9589 | outside |
| histstd | aisle | std | aisle | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| histstd | aisle | std | aisle | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 1.1577 | 0.9149 | inside | 19.1027 | 2.1026 | outside |
| histstd | aisle | std | aisle | yes (16 tracked envs) | same | 0.16-0.19 | 757 | -1.7173 | 2.215 | inside | 8.5865 | 2.3026 | outside |
| histstd | aisle | std | aisle | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 3.9947 | 1.8501 | outside | -0.2663 | 1.7167 | inside |
| histstd | aisle | std | aisle | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 1.0118 | 0.8258 | inside | -0.1686 | 0.7356 | inside |
| histstd | aisle | std | aisle | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| histstd | aisle | demohist | aisle | yes (16 tracked envs) | same | all | 2792 | -5.1218 | 0.7959 | outside | 3.4026 | 0.8744 | outside |
| histstd | aisle | demohist | aisle | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| histstd | aisle | demohist | aisle | yes (16 tracked envs) | same | 0.13-0.16 | 691 | -1.1577 | 1.0024 | inside | 3.1838 | 1.9392 | inside |
| histstd | aisle | demohist | aisle | yes (16 tracked envs) | same | 0.16-0.19 | 757 | -15.0594 | 2.2465 | outside | 8.9828 | 2.2036 | outside |
| histstd | aisle | demohist | aisle | yes (16 tracked envs) | same | 0.19-0.22 | 751 | -3.0626 | 1.5209 | outside | 1.0652 | 1.4591 | inside |
| histstd | aisle | demohist | aisle | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 0.3373 | 0.675 | inside | -0.5059 | 0.6533 | inside |
| histstd | aisle | demohist | aisle | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| histstd | aisle | acd3 | aisle | yes (16 tracked envs) | same | all | 2792 | 17.7292 | 0.9212 | outside | -12.1418 | 1.022 | outside |
| histstd | aisle | acd3 | aisle | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| histstd | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 0.7236 | 0.9711 | inside | 6.2229 | 1.9555 | outside |
| histstd | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 16.5125 | 1.9444 | outside | -11.3606 | 2.173 | outside |
| histstd | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 37.1505 | 2.1292 | outside | -32.2237 | 2.047 | outside |
| histstd | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 14.5025 | 1.6496 | outside | -9.1062 | 1.423 | outside |
| histstd | aisle | acd3 | aisle | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohist2 | demo1 | demohist | demo1 | yes (16 tracked envs) | same | all | 2792 | 2.4713 | 0.7405 | outside | -2.7937 | 0.7977 | outside |
| demohist2 | demo1 | demohist | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohist2 | demo1 | demohist | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 0.5789 | 0.9383 | inside | -1.5919 | 1.4828 | inside |
| demohist2 | demo1 | demohist | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 4.6235 | 1.8759 | outside | -4.7556 | 1.8801 | outside |
| demohist2 | demo1 | demohist | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 3.7284 | 1.4895 | outside | -3.9947 | 1.5004 | outside |
| demohist2 | demo1 | demohist | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 0.3373 | 1.2853 | inside | -0.1686 | 1.3182 | inside |
| demohist2 | demo1 | demohist | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demostd2 | demo1 | demostd | demo1 | yes (16 tracked envs) | same | all | 2792 | 2.7579 | 0.6805 | outside | -3.904 | 0.7242 | outside |
| demostd2 | demo1 | demostd | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demostd2 | demo1 | demostd | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 0.1447 | 0.7799 | inside | -4.9204 | 1.2486 | outside |
| demostd2 | demo1 | demostd | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 5.284 | 1.661 | outside | -5.284 | 1.661 | outside |
| demostd2 | demo1 | demostd | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 3.9947 | 1.4523 | outside | -4.1278 | 1.4579 | outside |
| demostd2 | demo1 | demostd | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 1.0118 | 1.2396 | inside | -0.6745 | 1.2627 | inside |
| demostd2 | demo1 | demostd | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohist2 | demo1 | demostd2 | demo1 | yes (16 tracked envs) | same | all | 2792 | 1.6476 | 0.7859 | outside | -1.8625 | 0.8515 | outside |
| demohist2 | demo1 | demostd2 | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohist2 | demo1 | demostd2 | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 1.3025 | 0.8553 | inside | -1.8813 | 1.5514 | inside |
| demohist2 | demo1 | demostd2 | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 4.0951 | 2.0123 | outside | -4.3593 | 2.0115 | outside |
| demohist2 | demo1 | demostd2 | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 1.5979 | 1.5979 | inside | -1.4647 | 1.6146 | inside |
| demohist2 | demo1 | demostd2 | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | -1.0118 | 1.4115 | inside | 0.8432 | 1.4217 | inside |
| demohist2 | demo1 | demostd2 | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohist2 | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | all | 2792 | 16.01 | 0.9365 | outside | -13.1447 | 1.0362 | outside |
| demohist2 | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohist2 | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 2.6049 | 0.8633 | outside | -1.1577 | 1.6759 | inside |
| demohist2 | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 23.5139 | 1.9966 | outside | -18.7583 | 2.1289 | outside |
| demohist2 | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 24.9001 | 2.1503 | outside | -22.5033 | 2.2095 | outside |
| demohist2 | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 10.7926 | 1.9617 | outside | -8.0944 | 2.054 | outside |
| demohist2 | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demostd2 | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | all | 2792 | 14.3625 | 0.9337 | outside | -11.2822 | 1.0116 | outside |
| demostd2 | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demostd2 | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 1.3025 | 0.8048 | inside | 0.7236 | 1.4837 | inside |
| demostd2 | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 19.4188 | 2.0067 | outside | -14.3989 | 2.0955 | outside |
| demostd2 | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 23.3023 | 2.1577 | outside | -21.0386 | 2.2086 | outside |
| demostd2 | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 11.8044 | 1.9663 | outside | -8.9376 | 2.0551 | outside |
| demostd2 | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohist2m | demo1 | demohist | demo1 | yes (16 tracked envs) | same | all | 2792 | 2.8295 | 0.71 | outside | -2.5072 | 0.7533 | outside |
| demohist2m | demo1 | demohist | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohist2m | demo1 | demohist | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | -0.2894 | 0.9159 | inside | 1.3025 | 1.3343 | inside |
| demohist2m | demo1 | demohist | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 5.4161 | 1.7375 | outside | -5.284 | 1.7431 | outside |
| demohist2m | demo1 | demohist | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 4.261 | 1.4141 | outside | -4.261 | 1.4141 | outside |
| demohist2m | demo1 | demohist | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 1.3491 | 1.37 | inside | -1.1804 | 1.4011 | inside |
| demohist2m | demo1 | demohist | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demostd2m | demo1 | demostd | demo1 | yes (16 tracked envs) | same | all | 2792 | 0.7521 | 0.6633 | inside | -1.2178 | 0.6998 | inside |
| demostd2m | demo1 | demostd | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demostd2m | demo1 | demostd | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 0.4342 | 0.8062 | inside | -2.3155 | 1.1732 | inside |
| demostd2m | demo1 | demostd | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 3.5667 | 1.5531 | outside | -3.5667 | 1.5531 | outside |
| demostd2m | demo1 | demostd | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | -0.2663 | 1.4226 | inside | 0.1332 | 1.4289 | inside |
| demostd2m | demo1 | demostd | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | -1.1804 | 1.2955 | inside | 1.3491 | 1.3278 | inside |
| demostd2m | demo1 | demostd | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohist2m | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | all | 2792 | 16.3682 | 0.9357 | outside | -12.8582 | 1.0188 | outside |
| demohist2m | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohist2m | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 1.7366 | 0.8419 | outside | 1.7366 | 1.5175 | inside |
| demohist2m | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 24.3065 | 2.0618 | outside | -19.2867 | 2.1553 | outside |
| demohist2m | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 25.4328 | 2.0917 | outside | -22.7696 | 2.157 | outside |
| demohist2m | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 11.8044 | 1.9372 | outside | -9.1062 | 2.0329 | outside |
| demohist2m | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demostd2m | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | all | 2792 | 12.3567 | 0.933 | outside | -8.596 | 1.009 | outside |
| demostd2m | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demostd2m | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 1.5919 | 0.8041 | inside | 3.3285 | 1.4643 | outside |
| demostd2m | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 17.7015 | 1.9972 | outside | -12.6816 | 2.081 | outside |
| demostd2m | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 19.0413 | 2.164 | outside | -16.7776 | 2.201 | outside |
| demostd2m | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 9.6121 | 2.0078 | outside | -6.914 | 2.0956 | outside |
| demostd2m | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohist2 | demo2 | demohist | demo2 | yes (16 tracked envs) | same | all | 2792 | 2.543 | 0.6957 | outside | -2.8653 | 0.7681 | outside |
| demohist2 | demo2 | demohist | demo2 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohist2 | demo2 | demohist | demo2 | yes (16 tracked envs) | same | 0.13-0.16 | 664 | 0.753 | 0.9646 | inside | -1.8072 | 1.4904 | inside |
| demohist2 | demo2 | demohist | demo2 | yes (16 tracked envs) | same | 0.16-0.19 | 752 | 5.5851 | 1.6387 | outside | -6.117 | 1.6683 | outside |
| demohist2 | demo2 | demohist | demo2 | yes (16 tracked envs) | same | 0.19-0.22 | 801 | 2.1223 | 1.3021 | inside | -2.1223 | 1.3721 | inside |
| demohist2 | demo2 | demohist | demo2 | yes (16 tracked envs) | same | 0.22-0.25 | 575 | 1.2174 | 1.5066 | inside | -0.8696 | 1.5854 | inside |
| demohist2 | demo2 | demohist | demo2 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demostd2 | demo2 | demostd | demo2 | yes (16 tracked envs) | same | all | 2792 | 1.6117 | 0.6431 | outside | -2.7579 | 0.7028 | outside |
| demostd2 | demo2 | demostd | demo2 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demostd2 | demo2 | demostd | demo2 | yes (16 tracked envs) | same | 0.13-0.16 | 664 | 0.6024 | 0.9995 | inside | -3.9157 | 1.3894 | outside |
| demostd2 | demo2 | demostd | demo2 | yes (16 tracked envs) | same | 0.16-0.19 | 752 | 5.3191 | 1.5165 | outside | -5.984 | 1.5421 | outside |
| demostd2 | demo2 | demostd | demo2 | yes (16 tracked envs) | same | 0.19-0.22 | 801 | 1.4981 | 1.2481 | inside | -1.623 | 1.3384 | inside |
| demostd2 | demo2 | demostd | demo2 | yes (16 tracked envs) | same | 0.22-0.25 | 575 | -1.913 | 1.1907 | inside | 1.2174 | 1.2174 | inside |
| demostd2 | demo2 | demostd | demo2 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohist2 | demo2 | demostd2 | demo2 | yes (16 tracked envs) | same | all | 2792 | 1.5759 | 0.7194 | outside | -1.2536 | 0.7869 | inside |
| demohist2 | demo2 | demostd2 | demo2 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohist2 | demo2 | demostd2 | demo2 | yes (16 tracked envs) | same | 0.13-0.16 | 664 | 0.753 | 0.9646 | inside | -0.4518 | 1.5443 | inside |
| demohist2 | demo2 | demostd2 | demo2 | yes (16 tracked envs) | same | 0.16-0.19 | 752 | 3.1915 | 1.7715 | inside | -2.7926 | 1.7873 | inside |
| demohist2 | demo2 | demostd2 | demo2 | yes (16 tracked envs) | same | 0.19-0.22 | 801 | 1.4981 | 1.3901 | inside | -1.1236 | 1.4292 | inside |
| demohist2 | demo2 | demostd2 | demo2 | yes (16 tracked envs) | same | 0.22-0.25 | 575 | 0.5217 | 1.3593 | inside | -0.3478 | 1.414 | inside |
| demohist2 | demo2 | demostd2 | demo2 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohist2 | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | all | 2792 | 15.0072 | 0.8387 | outside | -8.6676 | 0.9986 | outside |
| demohist2 | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohist2 | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.13-0.16 | 664 | 3.4639 | 0.9068 | outside | 2.1084 | 1.7686 | inside |
| demohist2 | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.16-0.19 | 752 | 23.1383 | 1.7939 | outside | -16.3564 | 1.9955 | outside |
| demohist2 | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.19-0.22 | 801 | 21.2235 | 1.7751 | outside | -13.8577 | 2.036 | outside |
| demohist2 | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.22-0.25 | 575 | 9.0435 | 1.8198 | outside | -3.8261 | 2.0086 | inside |
| demohist2 | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demostd2 | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | all | 2792 | 13.4312 | 0.8345 | outside | -7.414 | 0.9821 | outside |
| demostd2 | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demostd2 | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.13-0.16 | 664 | 2.7108 | 0.8981 | outside | 2.5602 | 1.6821 | inside |
| demostd2 | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.16-0.19 | 752 | 19.9468 | 1.8152 | outside | -13.5638 | 2.01 | outside |
| demostd2 | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.19-0.22 | 801 | 19.7253 | 1.7791 | outside | -12.7341 | 2.0336 | outside |
| demostd2 | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.22-0.25 | 575 | 8.5217 | 1.7651 | outside | -3.4783 | 1.9171 | inside |
| demostd2 | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohist2 | aisle | demohist | aisle | yes (16 tracked envs) | same | all | 2792 | 2.7221 | 0.7182 | outside | -2.2923 | 0.8047 | outside |
| demohist2 | aisle | demohist | aisle | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohist2 | aisle | demohist | aisle | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 1.1577 | 1.0633 | inside | 0.2894 | 1.8204 | inside |
| demohist2 | aisle | demohist | aisle | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 5.0198 | 2.0311 | outside | -4.4914 | 2.0241 | outside |
| demohist2 | aisle | demohist | aisle | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 4.261 | 1.2685 | outside | -4.3941 | 1.2749 | outside |
| demohist2 | aisle | demohist | aisle | yes (16 tracked envs) | same | 0.22-0.25 | 593 | -0.3373 | 0.7547 | inside | 0.1686 | 0.7356 | inside |
| demohist2 | aisle | demohist | aisle | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demostd2 | aisle | demostd | aisle | yes (16 tracked envs) | same | all | 2792 | 3.6891 | 0.6559 | outside | -5.3009 | 0.7058 | outside |
| demostd2 | aisle | demostd | aisle | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demostd2 | aisle | demostd | aisle | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 1.013 | 0.9036 | inside | -8.8278 | 1.4742 | outside |
| demostd2 | aisle | demostd | aisle | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 10.9643 | 1.8502 | outside | -10.4359 | 1.8257 | outside |
| demostd2 | aisle | demostd | aisle | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 2.53 | 1.2536 | outside | -1.8642 | 1.2047 | inside |
| demostd2 | aisle | demostd | aisle | yes (16 tracked envs) | same | 0.22-0.25 | 593 | -1.0118 | 0.4113 | outside | 1.0118 | 0.4113 | outside |
| demostd2 | aisle | demostd | aisle | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohist2 | aisle | demostd2 | aisle | yes (16 tracked envs) | same | all | 2792 | 1.6476 | 0.714 | outside | -1.6117 | 0.8187 | inside |
| demohist2 | aisle | demostd2 | aisle | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohist2 | aisle | demostd2 | aisle | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 0.8683 | 1.0028 | inside | -0.7236 | 1.9156 | inside |
| demohist2 | aisle | demostd2 | aisle | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 4.3593 | 2.0799 | outside | -4.0951 | 2.0806 | inside |
| demohist2 | aisle | demostd2 | aisle | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 1.8642 | 1.2622 | inside | -1.9973 | 1.2549 | inside |
| demohist2 | aisle | demostd2 | aisle | yes (16 tracked envs) | same | 0.22-0.25 | 593 | -1.1804 | 0.5577 | outside | 1.0118 | 0.5321 | inside |
| demohist2 | aisle | demostd2 | aisle | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohist2 | aisle | acd3 | aisle | yes (16 tracked envs) | same | all | 2792 | 25.5731 | 0.946 | outside | -17.8367 | 1.0476 | outside |
| demohist2 | aisle | acd3 | aisle | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohist2 | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 3.0391 | 1.0277 | outside | 3.3285 | 2.1098 | inside |
| demohist2 | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 36.5918 | 2.0802 | outside | -24.8349 | 2.2092 | outside |
| demohist2 | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 44.474 | 2.0007 | outside | -37.6831 | 1.9869 | outside |
| demohist2 | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 13.828 | 1.6421 | outside | -8.4317 | 1.43 | outside |
| demohist2 | aisle | acd3 | aisle | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demostd2 | aisle | acd3 | aisle | yes (16 tracked envs) | same | all | 2792 | 23.9255 | 1.0019 | outside | -16.2249 | 1.072 | outside |
| demostd2 | aisle | acd3 | aisle | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demostd2 | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 2.1708 | 0.9006 | outside | 4.0521 | 1.9477 | outside |
| demostd2 | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 32.2325 | 2.3099 | outside | -20.7398 | 2.3365 | outside |
| demostd2 | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 42.6099 | 2.1961 | outside | -35.6858 | 2.1828 | outside |
| demostd2 | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 15.0084 | 1.6678 | outside | -9.4435 | 1.4392 | outside |
| demostd2 | aisle | acd3 | aisle | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohistdyn | demo1 | demohist | demo1 | yes (16 tracked envs) | same | all | 2792 | -5.086 | 0.7756 | outside | 6.1963 | 0.8124 | outside |
| demohistdyn | demo1 | demohist | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohistdyn | demo1 | demohist | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | -1.7366 | 0.7635 | outside | 6.0781 | 1.2069 | outside |
| demohistdyn | demo1 | demohist | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | -8.4544 | 1.844 | outside | 8.5865 | 1.8479 | outside |
| demohistdyn | demo1 | demohist | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | -5.3262 | 1.7264 | outside | 5.3262 | 1.7264 | outside |
| demohistdyn | demo1 | demohist | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | -4.3845 | 1.4797 | outside | 4.3845 | 1.5177 | outside |
| demohistdyn | demo1 | demohist | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohistdyn | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | all | 2792 | 8.4527 | 0.9049 | outside | -4.1547 | 0.9753 | outside |
| demohistdyn | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohistdyn | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 0.2894 | 0.6476 | inside | 6.5123 | 1.312 | outside |
| demohistdyn | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 10.4359 | 1.8542 | outside | -5.4161 | 1.91 | outside |
| demohistdyn | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 15.8455 | 2.19 | outside | -13.1824 | 2.2374 | outside |
| demohistdyn | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 6.0708 | 2.0099 | outside | -3.5413 | 2.0962 | inside |
| demohistdyn | demo1 | acd3 | demo1 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohistdyn | demo2 | demohist | demo2 | yes (16 tracked envs) | same | all | 2792 | -4.0115 | 0.7196 | outside | 5.5516 | 0.7636 | outside |
| demohistdyn | demo2 | demohist | demo2 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohistdyn | demo2 | demohist | demo2 | yes (16 tracked envs) | same | 0.13-0.16 | 664 | -1.3554 | 0.9636 | inside | 5.7229 | 1.3466 | outside |
| demohistdyn | demo2 | demohist | demo2 | yes (16 tracked envs) | same | 0.16-0.19 | 752 | -8.9096 | 1.6568 | outside | 9.3085 | 1.6699 | outside |
| demohistdyn | demo2 | demohist | demo2 | yes (16 tracked envs) | same | 0.19-0.22 | 801 | -4.4944 | 1.4373 | outside | 5.2434 | 1.4557 | outside |
| demohistdyn | demo2 | demohist | demo2 | yes (16 tracked envs) | same | 0.22-0.25 | 575 | 0.0 | 1.477 | inside | 0.8696 | 1.5467 | inside |
| demohistdyn | demo2 | demohist | demo2 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohistdyn | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | all | 2792 | 8.4527 | 0.809 | outside | -0.2507 | 0.9389 | inside |
| demohistdyn | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohistdyn | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.13-0.16 | 664 | 1.3554 | 0.8375 | inside | 9.6386 | 1.4907 | outside |
| demohistdyn | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.16-0.19 | 752 | 8.6436 | 1.571 | outside | -0.9309 | 1.77 | inside |
| demohistdyn | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.19-0.22 | 801 | 14.6067 | 1.8776 | outside | -6.4919 | 2.0702 | outside |
| demohistdyn | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | 0.22-0.25 | 575 | 7.8261 | 1.8046 | outside | -2.087 | 1.9674 | inside |
| demohistdyn | demo2 | acd3 | demo2 | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohistdyn | aisle | demohist | aisle | yes (16 tracked envs) | same | all | 2792 | -5.8381 | 0.7284 | outside | 7.7006 | 0.8046 | outside |
| demohistdyn | aisle | demohist | aisle | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohistdyn | aisle | demohist | aisle | yes (16 tracked envs) | same | 0.13-0.16 | 691 | -1.7366 | 0.8903 | inside | 8.8278 | 1.636 | outside |
| demohistdyn | aisle | demohist | aisle | yes (16 tracked envs) | same | 0.16-0.19 | 757 | -17.3052 | 1.9442 | outside | 17.7015 | 1.953 | outside |
| demohistdyn | aisle | demohist | aisle | yes (16 tracked envs) | same | 0.19-0.22 | 751 | -2.6631 | 1.4806 | inside | 2.7963 | 1.4862 | inside |
| demohistdyn | aisle | demohist | aisle | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 0.0 | 0.7916 | inside | -0.1686 | 0.7734 | inside |
| demohistdyn | aisle | demohist | aisle | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |
| demohistdyn | aisle | acd3 | aisle | yes (16 tracked envs) | same | all | 2792 | 17.0129 | 0.9877 | outside | -7.8438 | 1.0553 | outside |
| demohistdyn | aisle | acd3 | aisle | yes (16 tracked envs) | same | < 0.13 | 0 |  |  |  |  |  |  |
| demohistdyn | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.13-0.16 | 691 | 0.1447 | 0.8809 | inside | 11.8669 | 1.8562 | outside |
| demohistdyn | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.16-0.19 | 757 | 14.2668 | 2.1905 | outside | -2.642 | 2.178 | inside |
| demohistdyn | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.19-0.22 | 751 | 37.5499 | 2.3006 | outside | -30.4927 | 2.2737 | outside |
| demohistdyn | aisle | acd3 | aisle | yes (16 tracked envs) | same | 0.22-0.25 | 593 | 14.1653 | 1.6717 | outside | -8.769 | 1.4464 | outside |
| demohistdyn | aisle | acd3 | aisle | yes (16 tracked envs) | same | >= 0.25 | 0 |  |  |  |  |  |  |

## 7. 건너는 사람

eval.json의 by_crossing: 나온 사람의 건넘 구간이 전역 경로를 덮음(경로 위), 덮지 않음(경로 밖), 건넘 없음으로 나눈 결과 비율. 충돌 경로 위-밖 차이는 서로 다른 에피소드 사이의 차이라 짝 차이가 아니며, SE는 두 비율의 독립 SE다. 에피소드가 없는 무리는 빈칸.

| 정책 | 칸 | 건넘 경로 위 수 | 건넘 경로 위 성공 % | 건넘 경로 위 충돌 % | 건넘 경로 위 시간 초과 % | 건넘 경로 밖 수 | 건넘 경로 밖 성공 % | 건넘 경로 밖 충돌 % | 건넘 경로 밖 시간 초과 % | 건넘 없음 수 | 건넘 없음 성공 % | 건넘 없음 충돌 % | 건넘 없음 시간 초과 % | 충돌 경로 위-밖 %p | 그 SE | 2SE |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| acd3 | aisle | 0 |  |  |  | 0 |  |  |  | 4096 | 56.62 | 35.62 | 7.76 |  |  |  |
| acd3 | aislet | 0 |  |  |  | 0 |  |  |  | 4096 | 56.64 | 35.52 | 7.84 |  |  |  |
| acd3 | demo1 | 2122 | 21.44 | 74.69 | 3.86 | 1140 | 89.39 | 9.65 | 0.96 | 834 | 37.65 | 55.76 | 6.59 | 65.04 | 1.2867 | outside |
| acd3 | demo1c08 | 2122 | 20.59 | 72.81 | 6.6 | 1140 | 90.0 | 8.86 | 1.14 | 834 | 43.76 | 48.44 | 7.79 | 63.95 | 1.2811 | outside |
| acd3 | demo1t | 2122 | 21.58 | 74.46 | 3.96 | 1140 | 89.74 | 9.21 | 1.05 | 834 | 37.53 | 56.12 | 6.35 | 65.25 | 1.2766 | outside |
| acd3 | demo2 | 2017 | 15.22 | 76.85 | 7.93 | 1009 | 89.99 | 9.51 | 0.5 | 1070 | 40.75 | 51.4 | 7.85 | 67.34 | 1.3172 | outside |
| ctrl | aisle | 0 |  |  |  | 0 |  |  |  | 4096 | 55.35 | 35.28 | 9.38 |  |  |  |
| ctrl | demo1 | 2122 | 21.44 | 74.98 | 3.58 | 1140 | 90.44 | 8.6 | 0.96 | 834 | 34.65 | 59.71 | 5.64 | 66.38 | 1.2544 | outside |
| ctrl | demo2 | 2017 | 15.02 | 79.38 | 5.6 | 1009 | 89.99 | 9.61 | 0.4 | 1070 | 38.79 | 54.67 | 6.54 | 69.77 | 1.2932 | outside |
| demo | aisle | 0 |  |  |  | 0 |  |  |  | 4096 | 55.98 | 39.89 | 4.13 |  |  |  |
| demo | demo1 | 2122 | 18.33 | 80.68 | 0.99 | 1140 | 89.04 | 10.53 | 0.44 | 834 | 36.21 | 62.71 | 1.08 | 70.15 | 1.2494 | outside |
| demo | demo2 | 2017 | 14.53 | 83.49 | 1.98 | 1009 | 88.6 | 11.0 | 0.4 | 1070 | 39.72 | 57.38 | 2.9 | 72.49 | 1.286 | outside |
| demo | head | 0 |  |  |  | 0 |  |  |  | 4096 | 45.07 | 54.39 | 0.54 |  |  |  |
| demo | rand | 0 |  |  |  | 0 |  |  |  | 4096 | 75.02 | 24.58 | 0.39 |  |  |  |
| demohist | aisle | 0 |  |  |  | 0 |  |  |  | 4096 | 72.19 | 25.02 | 2.78 |  |  |  |
| demohist | aislel958 | 0 |  |  |  | 0 |  |  |  | 4096 | 73.27 | 23.8 | 2.93 |  |  |  |
| demohist | aislepost | 0 |  |  |  | 0 |  |  |  | 4096 | 67.68 | 28.88 | 3.44 |  |  |  |
| demohist | aislepostm | 0 |  |  |  | 0 |  |  |  | 4096 | 33.94 | 63.7 | 2.37 |  |  |  |
| demohist | aislepostmf | 0 |  |  |  | 0 |  |  |  | 4096 | 71.8 | 25.34 | 2.86 |  |  |  |
| demohist | aislepostn | 0 |  |  |  | 0 |  |  |  | 4096 | 71.83 | 25.2 | 2.98 |  |  |  |
| demohist | aislet | 0 |  |  |  | 0 |  |  |  | 4096 | 72.22 | 24.9 | 2.88 |  |  |  |
| demohist | demo1 | 2122 | 34.54 | 63.85 | 1.6 | 1140 | 88.95 | 11.05 | 0.0 | 834 | 49.88 | 48.44 | 1.68 | 52.8 | 1.3964 | outside |
| demohist | demo1c08 | 2122 | 32.66 | 65.88 | 1.46 | 1140 | 87.63 | 12.28 | 0.09 | 834 | 52.76 | 45.92 | 1.32 | 53.6 | 1.4157 | outside |
| demohist | demo1l958 | 2122 | 34.21 | 64.33 | 1.46 | 1140 | 88.86 | 11.14 | 0.0 | 834 | 51.56 | 47.24 | 1.2 | 53.19 | 1.3963 | outside |
| demohist | demo1maskw | 2122 | 6.88 | 91.99 | 1.13 | 1140 | 38.51 | 61.4 | 0.09 | 834 | 7.55 | 92.21 | 0.24 | 30.59 | 1.5576 | outside |
| demohist | demo1post | 2122 | 28.18 | 70.83 | 0.99 | 1140 | 88.6 | 11.14 | 0.26 | 834 | 46.64 | 51.8 | 1.56 | 59.69 | 1.3572 | outside |
| demohist | demo1postm | 2122 | 7.07 | 91.56 | 1.37 | 1140 | 40.7 | 59.3 | 0.0 | 834 | 8.51 | 91.25 | 0.24 | 32.26 | 1.5752 | outside |
| demohist | demo1postmf | 2122 | 32.85 | 65.32 | 1.84 | 1140 | 88.33 | 11.67 | 0.0 | 834 | 49.04 | 49.76 | 1.2 | 53.65 | 1.4042 | outside |
| demohist | demo1postn | 2122 | 33.13 | 65.5 | 1.37 | 1140 | 88.77 | 11.23 | 0.0 | 834 | 50.72 | 47.96 | 1.32 | 54.27 | 1.3926 | outside |
| demohist | demo1t | 2122 | 34.35 | 64.18 | 1.46 | 1140 | 88.77 | 11.23 | 0.0 | 834 | 49.28 | 49.16 | 1.56 | 52.95 | 1.3992 | outside |
| demohist | demo1tc26a60 | 2122 | 34.59 | 63.57 | 1.84 | 1140 | 90.26 | 9.74 | 0.0 | 834 | 49.88 | 48.8 | 1.32 | 53.83 | 1.3647 | outside |
| demohist | demo1tc26a90 | 2122 | 32.52 | 65.74 | 1.74 | 1140 | 89.3 | 10.7 | 0.0 | 834 | 50.48 | 48.32 | 1.2 | 55.04 | 1.3782 | outside |
| demohist | demo1tf15a60 | 2122 | 0.94 | 36.8 | 62.25 | 1140 | 89.82 | 7.63 | 2.54 | 834 | 13.67 | 14.99 | 71.34 | 29.17 | 1.3093 | outside |
| demohist | demo1tf20a30 | 2122 | 14.23 | 38.88 | 46.89 | 1140 | 83.95 | 7.72 | 8.33 | 834 | 24.58 | 12.35 | 63.07 | 31.16 | 1.3209 | outside |
| demohist | demo1tf20a60 | 2122 | 0.05 | 24.27 | 75.68 | 1140 | 57.63 | 14.21 | 28.16 | 834 | 8.51 | 10.79 | 80.7 | 10.06 | 1.3912 | outside |
| demohist | demo1tp30a60 | 2122 | 41.42 | 56.79 | 1.79 | 1140 | 88.86 | 11.14 | 0.0 | 834 | 50.12 | 48.08 | 1.8 | 45.65 | 1.4229 | outside |
| demohist | demo1tp30a90 | 2122 | 40.76 | 57.59 | 1.65 | 1140 | 88.95 | 11.05 | 0.0 | 834 | 49.4 | 49.04 | 1.56 | 46.54 | 1.4189 | outside |
| demohist | demo1tw26a60 | 2122 | 43.59 | 54.52 | 1.89 | 1140 | 91.32 | 8.6 | 0.09 | 834 | 48.32 | 50.0 | 1.68 | 45.92 | 1.3631 | outside |
| demohist | demo1tw26a60n | 2122 | 42.93 | 55.37 | 1.7 | 1140 | 91.84 | 8.16 | 0.0 | 834 | 48.2 | 50.24 | 1.56 | 47.21 | 1.3498 | outside |
| demohist | demo2 | 2017 | 26.23 | 72.14 | 1.64 | 1009 | 88.11 | 11.89 | 0.0 | 1070 | 52.24 | 45.89 | 1.87 | 60.25 | 1.4264 | outside |
| demohist | head | 0 |  |  |  | 0 |  |  |  | 4096 | 51.12 | 48.88 | 0.0 |  |  |  |
| demohist | rand | 0 |  |  |  | 0 |  |  |  | 4096 | 77.37 | 22.56 | 0.07 |  |  |  |
| demohist2 | aisle | 0 |  |  |  | 0 |  |  |  | 4096 | 73.95 | 23.56 | 2.49 |  |  |  |
| demohist2 | demo1 | 2122 | 35.77 | 62.49 | 1.74 | 1140 | 88.42 | 11.58 | 0.0 | 834 | 54.68 | 42.93 | 2.4 | 50.91 | 1.4152 | outside |
| demohist2 | demo2 | 2017 | 29.25 | 68.96 | 1.78 | 1009 | 88.31 | 11.69 | 0.0 | 1070 | 53.18 | 44.39 | 2.43 | 57.27 | 1.4437 | outside |
| demohist2 | head | 0 |  |  |  | 0 |  |  |  | 4096 | 49.27 | 50.71 | 0.02 |  |  |  |
| demohist2 | rand | 0 |  |  |  | 0 |  |  |  | 4096 | 78.54 | 21.44 | 0.02 |  |  |  |
| demohist2m | demo1 | 2122 | 36.43 | 62.25 | 1.32 | 1140 | 89.3 | 10.7 | 0.0 | 834 | 54.2 | 44.48 | 1.32 | 51.55 | 1.3948 | outside |
| demohistdyn | aisle | 0 |  |  |  | 0 |  |  |  | 4096 | 68.16 | 30.32 | 1.51 |  |  |  |
| demohistdyn | demo1 | 2122 | 29.17 | 70.26 | 0.57 | 1140 | 87.98 | 12.02 | 0.0 | 834 | 46.4 | 53.0 | 0.6 | 58.24 | 1.3829 | outside |
| demohistdyn | demo2 | 2017 | 23.1 | 76.55 | 0.35 | 1009 | 88.4 | 11.6 | 0.0 | 1070 | 47.57 | 52.15 | 0.28 | 64.95 | 1.3807 | outside |
| demohistdyn | rand | 0 |  |  |  | 0 |  |  |  | 4096 | 76.15 | 23.78 | 0.07 |  |  |  |
| demostd | aisle | 0 |  |  |  | 0 |  |  |  | 4096 | 70.41 | 28.17 | 1.42 |  |  |  |
| demostd | demo1 | 2122 | 32.66 | 66.73 | 0.61 | 1140 | 88.16 | 11.84 | 0.0 | 834 | 48.08 | 51.2 | 0.72 | 54.89 | 1.4007 | outside |
| demostd | demo2 | 2017 | 26.23 | 72.24 | 1.54 | 1009 | 88.11 | 11.89 | 0.0 | 1070 | 50.47 | 48.79 | 0.75 | 60.35 | 1.4257 | outside |
| demostd | head | 0 |  |  |  | 0 |  |  |  | 4096 | 48.1 | 51.86 | 0.05 |  |  |  |
| demostd | rand | 0 |  |  |  | 0 |  |  |  | 4096 | 77.51 | 22.41 | 0.07 |  |  |  |
| demostd2 | aisle | 0 |  |  |  | 0 |  |  |  | 4096 | 72.92 | 24.56 | 2.51 |  |  |  |
| demostd2 | demo1 | 2122 | 35.34 | 62.77 | 1.89 | 1140 | 88.86 | 11.05 | 0.09 | 834 | 50.36 | 48.32 | 1.32 | 51.72 | 1.4012 | outside |
| demostd2 | demo2 | 2017 | 27.62 | 70.05 | 2.33 | 1009 | 89.4 | 10.6 | 0.0 | 1070 | 52.24 | 45.51 | 2.24 | 59.45 | 1.4069 | outside |
| demostd2 | rand | 0 |  |  |  | 0 |  |  |  | 4096 | 78.61 | 21.34 | 0.05 |  |  |  |
| demostd2m | demo1 | 2122 | 33.41 | 65.46 | 1.13 | 1140 | 88.07 | 11.93 | 0.0 | 834 | 48.68 | 50.36 | 0.96 | 53.53 | 1.4097 | outside |
| histstd | aisle | 0 |  |  |  | 0 |  |  |  | 4096 | 68.65 | 27.34 | 4.0 |  |  |  |
| histstd | demo1 | 2122 | 32.0 | 66.35 | 1.65 | 1140 | 89.12 | 10.7 | 0.18 | 834 | 48.56 | 49.52 | 1.92 | 55.65 | 1.3749 | outside |
| histstd | demo2 | 2017 | 25.24 | 72.34 | 2.43 | 1009 | 89.89 | 10.01 | 0.1 | 1070 | 48.04 | 49.81 | 2.15 | 62.33 | 1.3729 | outside |
| histstd | head | 0 |  |  |  | 0 |  |  |  | 4096 | 49.27 | 50.73 | 0.0 |  |  |  |
| histstd | rand | 0 |  |  |  | 0 |  |  |  | 4096 | 77.98 | 21.92 | 0.1 |  |  |  |
| std | aisle | 0 |  |  |  | 0 |  |  |  | 4096 | 67.9 | 22.66 | 9.45 |  |  |  |
| std | aislet | 0 |  |  |  | 0 |  |  |  | 4096 | 67.94 | 22.56 | 9.5 |  |  |  |
| std | demo1 | 2122 | 32.99 | 62.3 | 4.71 | 1140 | 89.12 | 10.53 | 0.35 | 834 | 52.04 | 42.21 | 5.76 | 51.77 | 1.3904 | outside |
| std | demo1c08 | 2122 | 34.45 | 61.97 | 3.58 | 1140 | 89.82 | 9.47 | 0.7 | 834 | 54.08 | 41.85 | 4.08 | 52.5 | 1.3648 | outside |
| std | demo1t | 2122 | 33.03 | 62.25 | 4.71 | 1140 | 89.21 | 10.44 | 0.35 | 834 | 52.4 | 41.97 | 5.64 | 51.81 | 1.3884 | outside |
| std | demo2 | 2017 | 26.13 | 67.77 | 6.1 | 1009 | 89.3 | 10.31 | 0.4 | 1070 | 52.15 | 42.15 | 5.7 | 57.46 | 1.414 | outside |
