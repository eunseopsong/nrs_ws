function Percent = Status_check(DataSize,Current_iter_num)

if rem(Current_iter_num,round(DataSize/100))==0
    Percent = [round(100*(Current_iter_num/DataSize)),1];
else
    Percent = [round(100*(Current_iter_num/DataSize)),0];
end

end