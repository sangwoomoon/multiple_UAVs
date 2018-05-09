function [det] = determine_on_line(pt,line_BP_1, line_BP_2)

cnt = 0;
if (min(line_BP_1(1,1),line_BP_2(1,1))<=pt(1,1)) && (max(line_BP_1(1,1),line_BP_2(1,1))>=pt(1,1))
    cnt=cnt+1;
end
if (min(line_BP_1(1,2),line_BP_2(1,2))<=pt(1,2)) && (max(line_BP_1(1,2),line_BP_2(1,2))>=pt(1,2))
    cnt=cnt+1;
end
if (cnt == 2)
    det = 1;
else
    det = 0;
end