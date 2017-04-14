
clc;
clear all;
dir_src ='D:\chy\result\';
 for n=1:30
 
 img_origin = strcat('depth_image_buff_consumer',int2str(n)) 
 img_before =strcat('binary_self_before',int2str(n));
 img_after  =strcat('binary_self_after ',int2str(n)); 
  
 read_img_origin =imread([dir_src img_origin, '.png']); 
 read_img_before =imread([dir_src img_before, '.png']); 
 read_img_after  =imread([dir_src img_after,  '.png']); 
 
 
 subplot(3,1,1);
 imshow(read_img_origin );title('img-origin');
 subplot(3,1,2);
 imshow(read_img_before);title('img-before');
 subplot(3,1,3);
 imshow( read_img_after);title('img-after');
pause(0.5);
 end






