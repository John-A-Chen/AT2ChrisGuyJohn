%% checks if a line intersects a triangle
% O is the line's origin and D is its direction vector. l is the
% segment's length
% A, B and C are the vertices of the triangle
% Credit: https://stackoverflow.com/questions/42740765/intersection-between-line-and-triangle-in-3d

function intersects = intersectTriangle(O, D, l, A, B, C)
    E1 = B - A;
    E2 = C - A;
    N = cross(E1, E2);
    det = - D' * N;
    if det == 0
        intersects = false;
        return
    end
    AO = O - A;
    DAO = cross(AO, D);
    u = E2' * DAO / det;
    v = -E1' * DAO / det;
    t = AO' * N / det;
    intersects = t >= 0 && t * norm(D) <= l && u >= 0 && v >= 0 && u + v <= 1;
end