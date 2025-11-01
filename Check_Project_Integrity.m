% ========================================
% 项目完整性检查脚本
% ========================================
% 此脚本检查所有必需文件和依赖项是否就位

fprintf('\n');
fprintf('========================================\n');
fprintf('  项目完整性检查\n');
fprintf('========================================\n\n');

%% 1. 检查MATLAB版本
fprintf('[1/5] 检查MATLAB版本...\n');
ver_info = ver('MATLAB');
ver_year = str2double(ver_info.Release(2:5));
if ver_year >= 2021
    fprintf('   ✓ MATLAB版本: %s (满足要求 >= R2021b)\n\n', ver_info.Release);
else
    fprintf('   ✗ MATLAB版本过低: %s (需要 >= R2021b)\n\n', ver_info.Release);
end

%% 2. 检查工具箱
fprintf('[2/5] 检查必需工具箱...\n');
required_toolboxes = {
    'Phased Array System Toolbox'
    'Sensor Fusion and Tracking Toolbox'
    '5G Toolbox'
};

v = ver;
installed_toolboxes = {v.Name};
all_installed = true;

for i = 1:length(required_toolboxes)
    if any(strcmp(installed_toolboxes, required_toolboxes{i}))
        fprintf('   ✓ %s\n', required_toolboxes{i});
    else
        fprintf('   ✗ %s (缺失)\n', required_toolboxes{i});
        all_installed = false;
    end
end

if all_installed
    fprintf('   ✓ 所有工具箱已安装\n\n');
else
    fprintf('   ⚠ 部分工具箱缺失，可能无法运行\n\n');
end

%% 3. 检查核心文件
fprintf('[3/5] 检查核心文件...\n');

core_files = {
    'Channel_Simulation_and_Sensing_Data_Processing.m'
    'Performance_Evaluation.m'
    'Visualize_Model_Probabilities.m'
    'Run_Complete_Demo.m'
    'ISAC_Scenario.m'
    'FiveG_Waveform_Config.m'
};

all_core_exist = true;
for i = 1:length(core_files)
    if exist(core_files{i}, 'file')
        fprintf('   ✓ %s\n', core_files{i});
    else
        fprintf('   ✗ %s (缺失)\n', core_files{i});
        all_core_exist = false;
    end
end

if all_core_exist
    fprintf('   ✓ 所有核心文件存在\n\n');
else
    fprintf('   ✗ 部分核心文件缺失\n\n');
end

%% 4. 检查Supporting_Functions文件
fprintf('[4/5] 检查Supporting_Functions文件...\n');

support_files = {
    'Supporting_Functions/helperGetTargetTrajectories.m'
    'Supporting_Functions/helperConfigureTracker.m'
    'Supporting_Functions/helperInitIMM.m'
    'Supporting_Functions/helperFormatDetectionsForTracker.m'
    'Supporting_Functions/isacBistaticMeasurementFcn.m'
    'Supporting_Functions/isacBistaticMeasurementJacobianFcn.m'
    'Supporting_Functions/helperGetCartesianMeasurement.m'
};

all_support_exist = true;
for i = 1:length(support_files)
    if exist(support_files{i}, 'file')
        fprintf('   ✓ %s\n', support_files{i});
    else
        fprintf('   ✗ %s (缺失)\n', support_files{i});
        all_support_exist = false;
    end
end

if all_support_exist
    fprintf('   ✓ 所有Supporting_Functions文件存在\n\n');
else
    fprintf('   ✗ 部分Supporting_Functions文件缺失\n\n');
end

%% 5. 检查文档
fprintf('[5/5] 检查文档文件...\n');

doc_files = {
    'README.md'
    '快速使用指南.md'
    '技术创新说明.md'
    '项目完成报告.md'
    '项目概览.md'
};

all_doc_exist = true;
for i = 1:length(doc_files)
    if exist(doc_files{i}, 'file')
        fprintf('   ✓ %s\n', doc_files{i});
    else
        fprintf('   ⚠ %s (缺失，但不影响运行)\n', doc_files{i});
        all_doc_exist = false;
    end
end

if all_doc_exist
    fprintf('   ✓ 所有文档文件存在\n\n');
else
    fprintf('   ⚠ 部分文档缺失\n\n');
end

%% 总结
fprintf('========================================\n');
fprintf('  检查总结\n');
fprintf('========================================\n\n');

can_run = all_installed && all_core_exist && all_support_exist;

if can_run
    fprintf('✅ 项目完整性检查通过！\n');
    fprintf('✅ 可以安全运行项目\n\n');
    fprintf('推荐运行命令:\n');
    fprintf('   >> Run_Complete_Demo\n\n');
else
    fprintf('❌ 项目完整性检查失败\n');
    fprintf('❌ 请先解决上述问题\n\n');
    
    if ~all_installed
        fprintf('缺失工具箱解决方案:\n');
        fprintf('   1. 打开MATLAB附加功能管理器\n');
        fprintf('   2. 搜索并安装缺失的工具箱\n\n');
    end
    
    if ~all_core_exist || ~all_support_exist
        fprintf('缺失文件解决方案:\n');
        fprintf('   1. 检查文件是否在正确位置\n');
        fprintf('   2. 重新下载或创建缺失文件\n\n');
    end
end

fprintf('详细信息请参阅: 快速使用指南.md\n');
fprintf('========================================\n\n');
