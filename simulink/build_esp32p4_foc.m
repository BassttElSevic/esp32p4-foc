function build_esp32p4_foc()
% Rebuild esp32p4_foc.slx from the adjacent parameter and private source files.
folder = fileparts(mfilename('fullpath'));
if bdIsLoaded('esp32p4_foc') && strcmp(get_param('esp32p4_foc','Dirty'),'on')
    error('esp32p4_foc:UnsavedModel','请先保存或关闭当前模型的未保存修改。');
end
build_model(folder);
end
