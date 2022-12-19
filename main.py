# -*- coding: utf-8 -*-

import json
import numpy as np
import cv2
from matplotlib import pyplot as plt
from shapely import geometry
from shapely.geometry import Polygon


# Найти длину диагонали полигона
def length(el):
    return int(((el[1][1] - el[0][1]) ** 2 + (el[1][0] - el[0][0]) ** 2) ** 0.5)


def additional_check(cell_edge, obs_i, obs_i_p_1):
    # Проводим линию между текущей точкой препятствия и следующей проверяемой
    line1 = geometry.LineString([obs_i, obs_i_p_1])
    # Проводим линию для данного ребра ячейки
    line2 = geometry.LineString(cell_edge)
    # Определяем характер их пересечения
    return str(line1.intersection(line2))


# Определить принадлежность ячейки: full, mixed, empty
# cell_points - список 4 точек ячейки
# obstacle_edges список списков угловых точек препятствий
def cell_checker(cell_points, obst_points, empty):
    # Создаем ребра ячейки
    cell_edges = [[cell_points[0], cell_points[1]], [cell_points[1], cell_points[2]], [cell_points[2], cell_points[3]],
                  [cell_points[3], cell_points[0]]]

    # Создаем полигон из переданной ячейки
    poly_a = Polygon([cell_points[0], cell_points[1], cell_points[2], cell_points[3]])

    # Проверка для каждого препятствия
    for obs in obst_points:
        # Создаем полигон из препятствия
        poly_b = Polygon(obs)
        # Если препятствие целиком включает в себя переданную ячейку
        if poly_b.contains(poly_a):
            return 'full'
        # Если ячейка целиком включает в себя препятствие
        if poly_a.contains(poly_b):
            return 'mixed'

        # Для каждого из ребер ячейки
        for cell_edge in cell_edges:
            # Пройти по всем угловым точкам препятствия
            for i in range(len(obs)):
                # Последняя точка
                if i == len(obs) - 1:
                    intersect = additional_check(cell_edge, obs[i], obs[0])
                else:
                    intersect = additional_check(cell_edge, obs[i], obs[i + 1])
                # Пересечение в точке
                if 'POINT' in intersect:
                    return 'mixed'

    # Иначе это пустая ячейка
    empty.append(cell_points)
    return 'empty'


def check_cell(st, cell, obst_points, path):
    # Если ячейка содержит препятствие, то она подлежит дальнейшней обработке
    cell_type = cell_checker(cell, obst_points, path)
    if cell_type == 'mixed':
        st.append(cell)


# Покрыть сеткой на основе quadtree
def quadtree(cell_points, img, path):
    # Порядок точек в cell_points
    #        0 ______ 3
    #         |      |
    #         |      |
    #         |______|
    #        1        2

    # Итеративная реализация
    stack = []
    # Кладем в стэк изначально стартовые вершины - угловые вершины самой карты
    stack.append(cell_points)

    # Пока размер стэка не 0
    while len(stack):
        # Берем последний элемент списка - LIFO
        cell_points = stack.pop()

        # Точки покоординатно
        x0, y0 = cell_points[0]
        x1, y1 = cell_points[1]
        x2, y2 = cell_points[2]
        x3, y3 = cell_points[3]

        # Проверяем, что высота больше минимальной
        if int(y1 - y0) > 4:
            cv2.line(img, ((x3 + x0) // 2, y0), ((x3 + x0) // 2, y1), (0, 0, 0), 1)  # vertical line
            cv2.line(img, (x0, (y1 + y0) // 2), (x3, (y1 + y0) // 2), (0, 0, 0), 1)  # horizontal line

            # Первая ячейка - левый верхний квадрант
            cell_0 = [(x0, y0), (x0, (y1 + y0) // 2), ((x3 + x0) // 2, (y1 + y0) // 2), ((x3 + x0) // 2, y0)]
            check_cell(stack, cell_0, obstacle_points, path)

            # Вторая ячейка - левый нижний квадрант
            cell_1 = [(x0, (y1 + y0) // 2), (x1, y1), ((x3 + x0) // 2, y1), ((x3 + x0) // 2, (y1 + y0) // 2)]
            check_cell(stack, cell_1, obstacle_points, path)

            # Третья ячейка - правый нижний квадрант
            cell_2 = [((x3 + x0) // 2, (y1 + y0) // 2), ((x3 + x0) // 2, y1), (x2, y2), (x3, (y1 + y0) // 2)]
            check_cell(stack, cell_2, obstacle_points, path)

            # Третья ячейка - правый верхний квадрант
            cell_3 = [((x3 + x0) // 2, y0), ((x3 + x0) // 2, (y1 + y0) // 2), (x3, (y1 + y0) // 2), (x3, y3)]
            check_cell(stack, cell_3, obstacle_points, path)


def line_intersection(line1, line2):
    line1 = geometry.LineString(line1)
    line2 = geometry.LineString(line2)
    x = str(line1.intersection(line2))
    if 'POINT' in x:
        return True
    else:
        return False


def graph_generation(empty_cells):
    counter = 0
    graph = {}
    threshold = 200  ## Search bubble radius
    for cell in empty_cells:
        cell_edges = [[cell[0], cell[1]], [cell[1], cell[2]], [cell[2], cell[3]], [cell[3], cell[0]]]
        cell_mp = ((cell[3][0] + cell[0][0]) // 2, (cell[1][1] + cell[0][1]) // 2)  ## mid point of the cell
        graph[cell_mp] = {}

        for acell in empty_cells:
            if cell == acell:
                continue
            acell_mp = ((acell[3][0] + acell[0][0]) // 2, (acell[1][1] + acell[0][1]) // 2)  ## mid point of the acell
            distance = int(((cell_mp[1] - acell_mp[1]) ** 2 + (cell_mp[0] - acell_mp[0]) ** 2) ** 0.5)

            if distance < threshold:
                acell_edges = [[acell[0], acell[1]], [acell[1], acell[2]], [acell[2], acell[3]], [acell[3], acell[0]]]
                for i in cell_edges:
                    for e in acell_edges:
                        counter += 1
                        if line_intersection(i, e):
                            graph[cell_mp].update({acell_mp: distance})
                            break
                    else:
                        continue
                    break
    print(counter)
    return graph


def dijkstra_algo(graph, start, goal):
    shortest_distance = {}
    predecessor = {}
    unseenNodes = graph
    infinity = float('inf')
    path = []
    for node in unseenNodes:
        shortest_distance[node] = infinity
    shortest_distance[start] = 0
    while unseenNodes:
        minNode = None
        for node in unseenNodes:
            if minNode is None:
                minNode = node
            elif shortest_distance[node] < shortest_distance[minNode]:
                minNode = node

        for childNode, weight in graph[minNode].items():
            if weight + shortest_distance[minNode] < shortest_distance[childNode]:
                shortest_distance[childNode] = weight + shortest_distance[minNode]
                predecessor[childNode] = minNode
        unseenNodes.pop(minNode)
    currentNode = goal
    while currentNode != start:
        try:
            path.insert(0, currentNode)
            currentNode = predecessor[currentNode]
        except KeyError:
            print('Path not reachable')
            break
    #    print(shortest_distance)
    path.insert(0, start)
    if shortest_distance[goal] != infinity:
        return shortest_distance[goal], path


if __name__ == "__main__":
    image = np.ones((100, 100, 3))
    image = 255 * image

    rows = image.shape[0]
    cols = image.shape[1]

    # Выгрузка данных о препятствиях из программы-генератора
    with open('obstacles_3_17.json', 'r') as f:
        output = json.load(f)
        info = output[0]
        startPoint = [output[1]['x'], output[1]['y']]
        endPoint = [output[2]['x'], output[2]['y']]

        # Массив, хранящий координаты вершин полигонов
        obstacle_points = []

        for inf_dict in output:
            if inf_dict['type'] == 'polygon':
                arr = []
                [arr.append([point['x'], point['y']]) for point in inf_dict['points']]
                obstacle_points.append(arr)

    path = []
    quadtree([(0, 0), (0, rows), (cols, rows), (cols, 0)], image, path)  # 0-1-2-3
    print(path)

    resized_image = cv2.resize(image, (500, 500))
    for obst in obstacle_points:
        resized_image = cv2.fillPoly(resized_image, np.array([obst], np.int32)*5, (0, 0, 255))

    image_new = cv2.addWeighted(resized_image, 0.4, cv2.resize(image, (500, 500)), 1 - 0.4, 0)
    cv2.imshow('image', image_new)
    cv2.imwrite('grid.png', image_new)
    plt.imshow(image_new)
    plt.show()

    # graph = graph_generation(path)
    # shortest_distance, path = dijkstra_algo(graph, (2, 2), (98, 98))
    #
    # for i in range(len(path) - 1):
    #     cv2.line(image, path[i], path[i + 1], (0, 0, 255), 3)
    #
    # print('Shortest path = ', path)
    # print()
    # print('Shortest distance = ', shortest_distance)

    cv2.waitKey(0)
    cv2.destroyAllWindows()


