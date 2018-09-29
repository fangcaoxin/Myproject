function images = readImages()
    imageDir = fullfile('cube');
    imds = imageDatastore(imageDir);
    images = cell(1, numel(imds.Files));
    for i = 1: numel(imds.Files)
       I = readimage(imds, i);
       images{i} = rgb2gray(I);
    end
end