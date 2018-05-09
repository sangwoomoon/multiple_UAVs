function [det] = check_new_edge(edge_new, edge_prev)

% singular case를 막기 위한 방법.
% if abs(edge_prev(2,1)-edge_prev(1,1)) < 0.0000001
%     edge_prev(2,1) = edge_prev(2,1) + 0.0001;
% end

% 여기서 버그 발생. 수정 필요. (전체적으로 수정해야 할 듯..;;)

% point landmark 는 없애자.
if (edge_new(2,1) == edge_new(1,1)) && (edge_new(2,2) == edge_new(1,2))
    det = 1;
else

slp = (edge_prev(2,2)-edge_prev(1,2))/(edge_prev(2,1)-edge_prev(1,1));

if (abs(slp*edge_new(1,1)-slp*edge_prev(1,1)+edge_prev(1,2)-edge_new(1,2)) < 0.01) && (abs(slp*edge_new(2,1)-slp*edge_prev(1,1)+edge_prev(1,2)-edge_new(2,2)) < 0.01)
    if (determine_on_line(edge_new(1,:),edge_prev(1,:),edge_prev(2,:))) || (determine_on_line(edge_new(2,:),edge_prev(1,:),edge_prev(2,:)))
        det = 0;
    elseif (determine_on_line(edge_prev(1,:),edge_new(1,:),edge_new(2,:))) || (determine_on_line(edge_prev(2,:),edge_new(1,:),edge_new(2,:)))
        det = 0;
    end    
    % 같은 직선방정식을 취하는 경우에 대해서 코딩 필요.
else
    det = 1;
end

end