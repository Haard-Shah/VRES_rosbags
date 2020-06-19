%% ROSBAG EXTRACTION TOOL
% Script is to extract images from rosbags collected using mini autonomous
% car platform. Need to specify a source folder containing one or more
% rosbags and a destination folder for when images will be extracted to.
%
% Author: J. Mount
% Creation Date: 26th November 2019
% 
% Updated: 26th November 2019

close all;
clear all;
clc

%% USER SETUP - enter user variables

% Specify the absolute source folder path containing the rosbag(s) that 
% need to extract images from
src = '/home/james/Desktop/VRES/run_1/'; 

% Specify the absolute destination folder path for where wish to extract
% images to
dst = '/home/james/Desktop/VRES/run_1/';

%% EXTRACTION

rosbagFiles = dir([src, filesep, '*.bag']);

for ii = 1:size(rosbagFiles, 1)
    fprintf("Extracting Images from Bag: %s\n", rosbagFiles(ii).name);
    
    % read bag and get messages
    bag = rosbag([src, filesep, rosbagFiles(ii).name]);
    msgs = readMessages(bag);
    
    counts = struct();
    
    % loop through messages
    for jj = 1:bag.NumMessages
        fprintf("\tChecking message %d of %d\n", jj, bag.NumMessages);
        
        data = bag.MessageList(jj, :);
        
        if strcmp(char(bag.MessageList.MessageType(jj)), 'sensor_msgs/Image')
            % read image
            [img, alpha] = readImage(msgs{jj});
            
            % get topic name without leading slash if exist and change / to
            % underscores
            topicName = strtrim(strrep(char(bag.MessageList.Topic(jj)), '/', ' '));
            topicName = strrep(topicName, ' ', '_');
            
            % add topic to counts
            if ~isfield(counts, topicName)
                counts.(topicName) = 0;
            end
            
            % get save directory and filename
            saveDir = [dst, filesep, topicName];
            imgName = sprintf('%s_frame_%06d', strrep(rosbagFiles(ii).name, '.bag', ''), counts.(topicName)); 
            
            % check to see if folder already exists
            if ~exist(saveDir, 'dir')
                mkdir(saveDir);
            end
            
            if isa(img, 'uint8')
                % save as png
                imwrite(img, [saveDir, filesep, imgName, '.png'], 'png');       
            elseif isa(img, 'uint16')
                % save as tif
                imwrite(img, [saveDir, filesep, imgName, '.tiff'], 'tiff'); 
            end
            
            % increase topic count by 1
            counts.(topicName) = counts.(topicName) + 1;
        end
    end
end

fprintf("COMPLETED\n\n");