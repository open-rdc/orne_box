# iias_tsukuba2024
## ソフトウェア構成
- ROS (Noetic)
- 自己位置推定 (emcl2)
- local_planner (DWAPlannerROS)
- global_planner (dijkstra)

## ros package (branch)
- [orne_box (TC_2024_EX)](https://github.com/open-rdc/orne-box/tree/TC_2024_EX)
move_baseを含む自律移動に必要なものがまとめてあるパッケージ
- [map_changer(master)](https://github.com/open-rdc/map_changer/tree/8ec95979d6ce3147d2514ee4d2d12c18706bac8a)
フロア移動をする際に，フロアに応じて占有格子地図を切り替えるパッケージ
- [waypoint_manager(main)](https://github.com/open-rdc/waypoint_manager/tree/v2.1.3)
waypoint関連のさまざまな機能を提供するパッケージ

# orne-box
Platform hardware for autonomous robot

* [Purpose](https://github.com/open-rdc/orne_box/wiki/Initial-Purpose)
* [Design Data (Under Constructing)](https://drive.google.com/drive/folders/1FTzKjHyfmug_UDPVUtk7wh9Z_zvEPqiV?usp=sharing)
* This project is derived from [orne_navigation](https://github.com/open-rdc/orne_navigation).

![image](https://user-images.githubusercontent.com/5755200/76318342-eb89c780-6320-11ea-900b-02a052fb53ae.png)

![DSC_0245](https://user-images.githubusercontent.com/5755200/80554308-b0923f00-8a07-11ea-80c8-d2e2097a1d2a.jpg)

# Reference
* [orne-x](https://drive.google.com/drive/folders/1ViINGsmbruIFg-iK9aN-tVQHTLGuMvhR?usp=sharing) (designed in 2017)
