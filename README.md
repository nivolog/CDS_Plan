#Описание
Данный пакет выполняет задачу планирования для системы управления Husky. Планировщик встроен в фреймворк ROS и представляет собой ROS-узел. 
##Системные требования

* Linux 18 и выше
* ROS Melodic (поддержка более поздних версий не гарантируется)

##Запуск

Запуск осуществляется при помощи утилиты roslaunch командой:

* roslaunch planner_cds planner_cds.launch 

##Конфигурация

Настройка пакета осуществляется при помощи конфигурационного файла config.yaml в директории src/planner_cs/Config. В данной версии доступны 4 параметра:

 * odom_topic - топик, в который должна публиковаться одометрия
 * task_topic - топик, в который должна публиковаться конечная точка
 * grid_topic - топик, в который должна публиковаться карта проходимости 
 * path_topic - топик, в который должен публиковаться результирующий путь 

##Особенности

Пакет поддерживает несколько планировщиков. Все они взяты из оригинального открытого пакета https://github.com/PathPlanning/AStar-JPS-ThetaStar. Таким образом поддерживаются планировщики:

* BFS
* Dijkstra
* AStar
* JPS
* ThetaStar


