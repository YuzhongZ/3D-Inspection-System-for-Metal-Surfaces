import cv2
import numpy as np
from matplotlib import pyplot as plt
import open3d as o3d
from PIL import Image
import tifffile as tiff

#file_path='D:/University34/capstone/1/data3_V0.05.pcd'
# path='D:/University34/capstone/Project2/Project2/小鼠标垫斜了/0_color.pcd'
# pcd = o3d.io.read_point_cloud(path)
# o3d.visualization.draw_geometries([pcd])

file_path='D:/University34/capstone/Project2/Project2/1111111_line.txt'
#data = np.genfromtxt(file_path,dtype=[float, float,float,float,])  # 将文件中数据加载到data数组里
data = np.genfromtxt(file_path,dtype=[float,],delimiter=',')
file_path2='D:/University34/capstone/Project2/Project2/2222222_line.txt'
data2 = np.genfromtxt(file_path2,dtype=[float,],delimiter=',')
file_path3='D:/University34/capstone/Project2/Project2/5555555_line.txt'
data3 = np.genfromtxt(file_path3,dtype=[float,],delimiter=',')
#print(data)
length=np.zeros(20)
aclen=np.zeros(20)
wifth=np.zeros(20)
acwid=np.zeros(20)
length2=np.zeros(20)
aclen2=np.zeros(20)
wifth2=np.zeros(20)
acwid2=np.zeros(20)
x=[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
t1=0
w1=0
t2=0
w2=0
for i in range(0,39,2):
  length[t1]=(data[i][0]+data[i+1][0])*0.5
  wifth[t1]=(data2[i][0]+data2[i+1][0])*0.5
  #length2[t1]=(data3[i][0]+data3[i+1][0])*0.5

  t1=t1+1
  # length[i]=data[i][0]
  # t1=t1+length[i]
  # aclen[i]=data[i][1]
  # wifth[i]=data[i][2]
  # w1=w1+wifth[i]
  # acwid[i]=data[i][3]
  # length2[i]=data2[i][0]
  # t2=t2+length2[i]
  # aclen2[i]=data2[i][1]
  # wifth2[i]=data2[i][2]
  # w2=w2+wifth2[i]
  # acwid2[i]=data2[i][3]

#plt.subplot(2,2,1)
plt.plot(x, length, 'b*--', alpha=0.5, linewidth=1, label='Sample1')
plt.plot(x, wifth, 'rs--', alpha=0.5, linewidth=1, label='Sample2')
#plt.plot(x, length2, 'go--', alpha=0.5, linewidth=1, label='Sample3')
plt.ylim(1.0,1.6)
plt.xlabel('time')
plt.ylabel('Straightness/mm')#accuracy
plt.legend()  #显示上面的label
# plt.subplot(2,2,2)
# plt.plot(x, wifth, 'b*--', alpha=0.5, linewidth=1, label='Sample1')
# #plt.ylim(167.3,168.3)
# plt.xlabel('time')
# plt.ylabel('Width/mm')#accuracy
# plt.legend()  #显示上面的label
# #plt.ylim(-1,1)#仅设置y轴坐标范围
# plt.subplot(2,2,3)
# plt.plot(x, length2, 'rs--', alpha=0.5, linewidth=1, label='Sample2')
# #plt.ylim(167.3,168.3)
# plt.xlabel('time')
# plt.ylabel('Width/mm')#accuracy
# plt.legend()  #显示上面的label
# plt.subplot(2,2,4)
# plt.plot(x, wifth2, 'rs--', alpha=0.5, linewidth=1, label='Sample2')#Acc of Width
# #plt.ylim(167.3,168.3)
# plt.xlabel('time')
# plt.ylabel('Width/mm')#accuracy
# plt.legend()  #显示上面的label
plt.show()


# img = cv2.imread('D:/University34/capstone/Project2/Project2/1111111/0Intensity.png',0)
# img2 = cv2.imread('D:/University34/capstone/Project2/Project2/6666666/0Intensity.png',0)
# img3 = cv2.imread('D:/University34/capstone/Project2/Project2/小鼠标垫斜了/0Intensity.png',0)
# otsu1 = cv2.imread('D:/University34/capstone/capstonepicture/otsu1.png',1)
# otsu2 = cv2.imread('D:/University34/capstone/capstonepicture/otsu2.png',1)
# otsu3 = cv2.imread('D:/University34/capstone/capstonepicture/otsu3.png',1)
# blur1 = cv2.GaussianBlur(img,(5,5),0)
# blur2 = cv2.GaussianBlur(img2,(5,5),0)
# blur3 = cv2.GaussianBlur(img3,(5,5),0)
# ret1,th1 = cv2.threshold(blur1,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
# ret2,th2 = cv2.threshold(blur2,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
# ret3,th3 = cv2.threshold(blur3,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
# images0 = [img, 
#           img2,
#           img3]
# ret=[ret1,ret2,ret3]
# # images = [img, 0, th1,
# #           img2, 0, th2,
# #           img3, 0, th3]
# images = [th1, 0, otsu1,
#           th2, 0, otsu2,
#           th3, 0, otsu3]
# titles = ['Normal Intensity','Histogram',"Point Cloud",
#           'Abnormal Intensity','Histogram',"Point Cloud",
#           'Abnormal Intensity(Different Background)','Histogram',"Point Cloud"]
# for i in range(3):
#     plt.subplot(3,3,i*3+1),plt.imshow(images[i*3],'gray')
#     plt.title(titles[i*3]), plt.xticks([]), plt.yticks([])
#     plt.subplot(3,3,i*3+2),plt.hist(images0[i].ravel(),256)
#     plt.axvline(ret[i],0, 20000, color='red',linestyle='dashed')
#     plt.title(titles[i*3+1]), 
#     plt.subplot(3,3,i*3+3),plt.imshow(images[i*3+2],'gray')
#     plt.title(titles[i*3+2]), plt.xticks([]), plt.yticks([])
# plt.show()
# global thresholding
# ret1,th1 = cv2.threshold(img,127,255,cv2.THRESH_BINARY)

# # Otsu's thresholding
# ret2,th2 = cv2.threshold(img,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

# # Otsu's thresholding after Gaussian filtering
# blur = cv2.GaussianBlur(img,(5,5),0)
# ret3,th3 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
# print(ret2)
# print(ret2)
# plot all the images and their histograms
# images = [img, 0, th1,
#           img, 0, th2,
#           blur, 0, th3]
# titles = ['Original Noisy Image','Histogram','Global Thresholding (v=127)',
#           'Original Noisy Image','Histogram',"Otsu's Thresholding",
#           'Gaussian filtered Image','Histogram',"Otsu's Thresholding"]
# # plt.hist(images[6].ravel(),256)
# # plt.show()
# for i in range(3):
#     plt.subplot(3,3,i*3+1),plt.imshow(images[i*3],'gray')
#     plt.title(titles[i*3]), plt.xticks([]), plt.yticks([])
#     plt.subplot(3,3,i*3+2),plt.hist(images[i*3].ravel(),256)
#     plt.title(titles[i*3+1]), plt.xticks([]), plt.yticks([])
#     plt.subplot(3,3,i*3+3),plt.imshow(images[i*3+2],'gray')
#     plt.title(titles[i*3+2]), plt.xticks([]), plt.yticks([])
# plt.show()



# if __name__ == '__main__':
 

#     # pcd = o3d.io.read_point_cloud(file_path)
#     # print(np.asarray(pcd.points))
#     # colors = np.asarray(pcd.colors) * 255
#     # points = np.asarray(pcd.points)
#     # print(points.shape, colors.shape)
#     # pointclouds=np.concatenate([points, colors], axis=-1)
# #    save_path='D:/University34/capstone/AST-main/AST-main/dataset/mvtec3d/mouse/test/good/gt/00'
# #    for i in range(10):
#     file_path='D:/University34/capstone/1/data3_V0.05.pcd'
# #      path='D:/University34/capstone/AST-main/AST-main/dataset/mvtec3d/mouse/test/good/xyz/00'+str(i)+'.tiff'
#     pcd = o3d.io.read_point_cloud(file_path)
#     o3d.visualization.draw_geometries([pcd])
#     pts = np.asarray(pcd.points)
#     # 根据高度生成色彩
#     colors = np.zeros([pts.shape[0], 3])
#     height_max = np.max(pts[:, 2])
#     height_min = np.min(pts[:, 2])
#     delta_c = abs(height_max - height_min) / (255 * 2)
#     for j in range(pts.shape[0]):
#        color_n = (pts[j, 2] - height_min) / delta_c
#        if color_n <= 255:
#           colors[j, :] = [0, 1 - color_n / 255, 1]
#        else:
#           colors[j, :] = [(color_n - 255) / 255, 0, 1]
 
#     pcd.colors = o3d.utility.Vector3dVector(colors)
#     o3d.visualization.draw_geometries([pcd], window_name="wechat 394467238 ")
 


 

 
#      tiff_img = tiff.imread(path)
#      height, width, _=tiff_img.shape
#       #print(height)
#       #print(width)
#     #  vis = o3d.visualization.VisualizerWithVertexSelection()
#     #  vis.create_window(window_name='Open3D', visible=True)
#     #  vis.add_geometry(pcd)
#     #  vis.run()
#     #  point = vis.get_picked_points()
#     #  vis.destroy_window()
#     #  tindex=np.zeros(len(point))
#      image=Image.new(mode='RGB',size=(width,height),color=(0,0,0))
#       #imapixel=image.load()
# #    image.show()
#     #  for j in range(len(point)):
#     #    tindex[j]=point[j].index
#     #    tempi=tindex[j]
#     #    row=int(tempi/width)
#     #    col=int(tempi%width)
#     #    image.putpixel((col,row),(255,255,255))
        
#         #x,y,z=np.asarray(point[0].coord)
#      image.save(save_path+str(i)+'.png')
#      #np.savetxt(save_path+str(i+1)+'_gt.csv', tindex, delimiter=',')
     
     # filename='D:/University34/capstone/ADS_main_3D/ADS_main_3D/pixel_preds0.txt'
     # path='D:/University34/capstone/ADS_main_3D/ADS_main_3D/gt_t/20t_gt.png'
     # image=Image.open(path)
     # #height, width, _=image.shape
     # #image.show()
     # imagegt=Image.new(mode='RGB',size=(224,224),color=(0,0,0))
     # data = np.loadtxt(filename,dtype=np.float32,delimiter=',')
     # #print(data)
     # num=0
     # for i in range(50176):
     #      #index=i+1003520
     #      row=int(i/224)
     #      col=int(i%224)
     #      if data[i+1003520]>8.005399227142333984e+00:
     #           imagegt.putpixel((col,row),(255,255,255))
     # imagegt.show()  
               
          # if i % 50176==0:
          #      path='D:/University34/capstone/Project2/Project2/未校正ads_datasets/dddorganized00/'+str(i)+'_tiff_00.tiff'
          #      num=num+1

          

   

#    print(point[0].index, np.asarray(point[0].coord))








