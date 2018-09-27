function pt = closestPoint(pos, v1, v2)
    A = pos - v1;
    B = v2 - v1;
    projection = (dot(A, B) / norm(B)^2)*B;
    pt = v1 + projection;
end