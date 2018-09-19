%disp('Saving the Images in folders. This might take some time...');    
%name = 'C:\Users\IceFox\Desktop\ERMINE\MATLAB\Machine_Learning\CIFAR10\cifar-10-batches-mat';
%saveCIFAR10AsFolderOfImages(name, pwd, true);


cifar10TrainPath = 'C:\Users\IceFox\Desktop\ERMINE\MATLAB\Machine_Learning\CIFAR10\cifar10Train';
cifar10TestPath = 'C:\Users\IceFox\Desktop\ERMINE\MATLAB\Machine_Learning\CIFAR10\cifar10Test';
%% Load image CIFAR-10 Training dataset (50000 32x32 colour images in 10 classes)
imsetTrain = imageSet(cifar10TrainPath,'recursive');

%% Display Sampling of Image Data
numClasses = size(imsetTrain,2);
imagesPerClass = 10;
imagesInMontage = cell(imagesPerClass,numClasses);
for i = 1:size(imagesInMontage,2)
    imagesInMontage(:,i) = ...
        imsetTrain(i).ImageLocation(randi(imsetTrain(i).Count, 1, ...
        imagesPerClass));
end

montage({imagesInMontage{:}},'Size',[numClasses,imagesPerClass]);
title('Sample of Training Data (Credit:Learning Multiple Layers of Features from Tiny Images, Alex Krizhevsky, 2009.)')

%% Prepare the data for Training
% Read all images and store them in a 4D uint8 input array for training,
% with its corresponding class

trainNames = {imsetTrain.Description};
XTrain = zeros(32,32,3,sum([imsetTrain.Count]),'uint8');
TTrain = categorical(discretize((1:sum([imsetTrain.Count]))',...
    [0,cumsum([imsetTrain.Count])],'categorical',trainNames));

j = 0;
tic;
for c = 1:length(imsetTrain)
    for i = 1:imsetTrain(c).Count
        XTrain(:,:,:,i+j) = read(imsetTrain(c),i);
    end
    j = j + imsetTrain(c).Count;
end
toc;

%% Define a CNN architecture
conv1 = convolution2dLayer(5,32,'Padding',2,...
                     'BiasLearnRateFactor',2);
conv1.Weights = gpuArray(single(randn([5 5 3 32])*0.0001));
fc1 = fullyConnectedLayer(64,'BiasLearnRateFactor',2);
fc1.Weights = gpuArray(single(randn([64 576])*0.1));
fc2 = fullyConnectedLayer(10,'BiasLearnRateFactor',2);
fc2.Weights = gpuArray(single(randn([10 64])*0.1));

layers = [ ...
    imageInputLayer([32 32 3]);
    conv1;
    maxPooling2dLayer(3,'Stride',2);
    reluLayer();
    convolution2dLayer(5,32,'Padding',2,'BiasLearnRateFactor',2);
    reluLayer();
    averagePooling2dLayer(3,'Stride',2);
    convolution2dLayer(5,64,'Padding',2,'BiasLearnRateFactor',2);
    reluLayer();
    averagePooling2dLayer(3,'Stride',2);
    fc1;
    reluLayer();
    fc2;
    softmaxLayer()
    classificationLayer()];

% Define the training options.
opts = trainingOptions('sgdm', ...
    'InitialLearnRate', 0.001, ...
    'LearnRateSchedule', 'piecewise', ...
    'LearnRateDropFactor', 0.1, ...
    'LearnRateDropPeriod', 8, ...
    'L2Regularization', 0.004, ...
    'MaxEpochs', 10, ...
    'MiniBatchSize', 100, ...
    'Verbose', true);

%% Training the CNN
[net, info] = trainNetwork(XTrain, TTrain, layers, opts);

% Alternative way using imageDataStore
% imdsTrain = imageDatastore(fullfile(pwd,'cifar10Train'),...
%     'IncludeSubfolders',true,'LabelSource','foldernames');
% [net, info] = trainNetwork(imdsTrain, layers, opts);

%%  Visualise the first layer weights.
figure;
montage(mat2gray(gather(net.Layers(2).Weights)));
title('First Layer Weights');

%% Load Test Data

imsetTest = imageSet(cifar10TestPath,'recursive');

testNames = {imsetTest.Description};
XTest = zeros(32,32,3,sum([imsetTest.Count]),'uint8');
TTest = categorical(discretize((1:sum([imsetTest.Count]))',...
    [0,cumsum([imsetTest.Count])],'categorical',testNames));
j = 0;
tic;
for c = 1:length(imsetTest)
    for i = 1:imsetTest(c).Count
        XTest(:,:,:,i+j) = read(imsetTest(c),i);
    end
    j = j + imsetTest(c).Count;
end
toc;

%% Run the network on the test set

YTest = classify(net, XTest);

% Alternative way using imageDataStore
% imdsTest = imageDatastore(fullfile(pwd, 'cifar10Test'),...
%     'IncludeSubfolders',true,'LabelSource','foldernames');
% YTest = classify(net, imdsTest);

% Calculate the accuracy.
accuracy = sum(YTest == TTest)/numel(TTest)