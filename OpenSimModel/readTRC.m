%% [Head1,Head2,data]=readTRC(fid)
% reads the marker trajectory file 
%%
function TRC=readTRC(fid)
for q=1:3
    tmp=fgets(fid);
end
TRC.Head1=strread(fgets(fid),'%s')';
TRC.Head2=strread(fgets(fid),'%s')';
TRC.data=[];
tmp=fgets(fid);
while ischar(tmp)
    %     data=[data;str2num(tmp)];%#ok
    tmp2=textscan(tmp,'%f','delimiter','\t');
    tmp2=tmp2{1};
    if ~isempty(tmp2)
        TRC.data(end+1,1:length(tmp2))=tmp2;%#ok
    end
    tmp=fgets(fid);
end