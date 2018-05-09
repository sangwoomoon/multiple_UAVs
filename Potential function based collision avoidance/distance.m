function dist = distance(A,B)

if (length(A) == 3)
    dist = sqrt((A(1)-B(1))^2+(A(2)-B(2))^2+(A(3)-B(3))^2);
else
    dist = sqrt((A(1)-B(1))^2+(A(2)-B(2))^2);
end