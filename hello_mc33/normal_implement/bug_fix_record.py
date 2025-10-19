from typing import List


def get_center_point(vertices: List[List[float]]):
    center_x = 0.0
    center_y = 0.0
    center_z = 0.0
    for vertice in vertices:
        center_x = center_x + vertice[0]
        center_y = center_y + vertice[1]
        center_z = center_z + vertice[2]
    center_x = center_x / len(vertices)
    center_y = center_y / len(vertices)
    center_z = center_z / len(vertices)
    return center_x, center_y, center_z


# NOTE: no such problem, there seems to be something wring with signed distance function! The discrepancy between two vertices' signed distance could not bigger than 1!
def find_center_point():
    """get an inner point of three triangles. These triangles are all on the generated mesh by mc33 tables, however, between them there is a blank which cause discontinuity visually. By my suscipition, it may be caused the wrong mc33 tables. So I get a point in this blank area and see which cube is handling this area
    conclusion: after check, its result is correct... may be it's just an unusual case?
    """
    vertices = {
        2145: [8, 17.0818, 7],
        2146: [9, 17, 7.56489],
        2147: [8.48115, 18, 7],
        2148: [8.52561, 17, 8],
        2149: [9, 17, 7.56489],
        2150: [8, 17.0818, 7],
        2151: [8, 18, 7.54014],
        2152: [8.52561, 17, 8],
        2153: [8, 17.0818, 7],
    }
    center_point1 = [
        (vertices[2145][0] + vertices[2146][0] + vertices[2147][0]) / 3,
        (vertices[2145][1] + vertices[2146][1] + vertices[2147][1]) / 3,
        (vertices[2145][2] + vertices[2146][2] + vertices[2147][2]) / 3,
    ]
    center_point2 = [
        (vertices[2148][0] + vertices[2149][0] + vertices[2150][0]) / 3,
        (vertices[2148][1] + vertices[2149][1] + vertices[2150][1]) / 3,
        (vertices[2148][2] + vertices[2149][2] + vertices[2150][2]) / 3,
    ]
    center_point3 = [
        (vertices[2151][0] + vertices[2152][0] + vertices[2153][0]) / 3,
        (vertices[2151][1] + vertices[2152][1] + vertices[2153][1]) / 3,
        (vertices[2151][2] + vertices[2152][2] + vertices[2153][2]) / 3,
    ]
    center_point = [
        (center_point1[0] + center_point2[0] + center_point3[0]) / 3,
        (center_point1[1] + center_point2[1] + center_point3[1]) / 3,
        (center_point1[2] + center_point2[2] + center_point3[2]) / 3,
    ]
    print(center_point)


def interpolation_test():
    A0 = -0.42194957453863824
    B0 = 0.0089563300454618688
    C0 = 0.44503042526993991
    D0 = 0.16540422949803105
    A1 = 0.067295211829972640
    B1 = -0.33561692565043932
    C1 = -0.23003444777424031
    D1 = -0.48955763832964500
    a = (A1 - A0) * (C1 - C0) - (B1 - B0)*(D1 - D0)
    b = C0*(A1 - A0) + A0*(C1 - C0) - D0*(B1 - B0) - B0*(D1 - D0)
    c = A0 * C0 - B0 * D0
    print(f"a={a}")
    print(f"b={b}")
    print(f"c={c}")
    print(f"-b/2a={-b/(2*a)}")

    def parabola(t):
        return a*t*t + b*t + c

    print(f"f(0)={parabola(0)}")
    print(f"f(1)={parabola(1)}")
    print(f"f(-b/2a)={parabola(-b/(2*a))}")

    print("--------------------------")
    print(f"57-46={A0*B1 - B0*A1}")
    print(f"52-16={A0*D1 - D0*A1}")


def find_center_point_2():
    vertices = {
        3207: [14.8792, 27, 10],
        3208: [14, 27, 9.25254],
        3209: [14.559, 28, 10],
        3210: [14.8792, 27, 10],
        3211: [15, 27, 9.86245],
        3212: [14, 27, 9.25254],
        3213: [14.2816, 27, 9],
        3214: [14, 27, 9.25254],
        3215: [15, 27, 9.86145],
        3219: [14.4362, 28, 9],
        3220: [14.2816, 27, 9],
        3221: [15, 27, 9.86245],
    }
    vertice_list = []
    for key, value in vertices.items():
        vertice_list.append(value)
    center_point = get_center_point(vertice_list)
    print(center_point)


if __name__ == "__main__":
    find_center_point_2()
