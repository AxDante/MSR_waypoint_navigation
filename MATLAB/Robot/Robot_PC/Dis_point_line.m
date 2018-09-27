function dis = Dis_point_line(pt, v1, v2)
      a = v1 - v2;
      b = pt - v2;
      dis = norm(cross(a,b)) / norm(a);