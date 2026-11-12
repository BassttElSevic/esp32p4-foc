function chart = mf_chart(parent,name,position,code,paramNames)
% Store the source and model-workspace parameter bindings in the SLX block.
path = [parent '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function',path,'Position',position);
root = sfroot;
chart = root.find('-isa','Stateflow.EMChart','Path',path);
chart.Script = char(code);
for i = 1:numel(paramNames)
    name = char(paramNames{i});
    datum = chart.find('-isa','Stateflow.Data','Name',name);
    if isempty(datum)
        datum = Stateflow.Data(chart);
        datum.Name = name;
    end
    datum.Scope = 'Parameter';
end
end
