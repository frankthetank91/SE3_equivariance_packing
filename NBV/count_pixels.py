from PIL import Image, ImageChops
import numpy as np
import cv2
import matplotlib.pyplot as plt

def count(rgb_image,depth_image):

    compare_depth = 0.35#0.20
    depth_count = np.count_nonzero(depth_image < compare_depth)
    print('valid depth pixels ', depth_count)
    print('rgb size ',rgb_image.shape)
    print('depth size', depth_image.shape)
    height,width = depth_image.shape[:2]
    total_count = height*width

    num_pixels = total_count - depth_count
    valid_pixel_percentage = num_pixels/total_count*100

    image_index = 0
    save_text_file(image_index,compare_depth,valid_pixel_percentage,num_pixels,depth_count,total_count)

    return num_pixels,depth_count,total_count,valid_pixel_percentage

def compare_count(empty_rgb_image,empty_depth_image,rgb_image,depth_image):

    image_index = 15

    # Find and plot the depth image difference 
    empty = empty_depth_image #np.int32((Image.open("depth_image0.png")).getdata())#empty_depth_image.getdata()
    full = depth_image #list((Image.open("depth_image1.png")).getdata())#Image.Image.getdata(depth_image)
    difference = np.abs(full-empty)

    plt.title("Depth Image Difference"); plt.xlabel("x"); plt.ylabel("y")
    plt.imshow(difference)
    plt.savefig(f'depth_image_difference{image_index}.png')
    #plt.show()

    plt.title("Empty Depth Image"); plt.xlabel("x"); plt.ylabel("y")
    plt.imshow(empty)
    plt.savefig(f'depth_image_empty{image_index}.png')
    #plt.show()

    plt.title("Full Depth Image"); plt.xlabel("x"); plt.ylabel("y")
    plt.imshow(full)
    plt.savefig(f'depth_image_full{image_index}.png')
    #plt.show()


    # Find pixel values for depth readings and calculate the sum
    height,width = depth_image.shape[:2]
    total_count = height*width
    depth_count = np.count_nonzero([(full[i]-empty[i])/total_count for i in range(len(full))])
    sum = np.sum(abs(full-empty))
    sum_whole_image = np.sum(abs(full-empty))/total_count
    norm_depth_count = depth_count/total_count
    norm_sum = sum/depth_count
    norm_sum_whole = sum/total_count

    print(total_count);print(depth_count);print(sum);print(sum_whole_image)
    print(norm_depth_count);print(norm_sum);print(norm_sum_whole)

    num_pixels = total_count - depth_count
    valid_pixel_percentage = num_pixels/total_count*100

   #print(full)

    save_text_file(image_index,valid_pixel_percentage,num_pixels,depth_count,total_count,sum,sum_whole_image,norm_depth_count,norm_sum,norm_sum_whole)

    return num_pixels,depth_count,total_count,sum,valid_pixel_percentage

def count_unique(image):
    unique = np.unique(image)
    return unique

def save_text_file(image_index,valid_pixel_percentage,num_pixels,depth_count,total_count,sum,sum_whole_image,norm_depth_count,norm_sum,norm_sum_whole):#,compare_depth):
    with open(f"PixelData{image_index}", "w") as file:
        file.write(f"image_index: {image_index}\n")
        #file.write(f"compare_depth: {compare_depth}\n")
        file.write(f"valid_pixel_percentage: {valid_pixel_percentage}\n")
        file.write(f"num_pixels: {num_pixels}\n")
        file.write(f"depth_count: {depth_count}\n")
        file.write(f"total_count: {total_count}\n")
        file.write(f"sum: {sum}\n")
        file.write(f"sum_whole_image: {sum_whole_image}\n")
        file.write(f"norm_depth_count: {norm_depth_count}\n")
        file.write(f"norm_sum: {norm_sum}\n")
        file.write(f"norm_sum_whole: {norm_sum_whole}\n")

# Load images
empty_rgb_image = cv2.imread("rgb_image14.png")
empty_depth_image = cv2.imread("depth_image14.png",cv2.IMREAD_UNCHANGED)
empty_depth_image_gray = cv2.cvtColor(empty_depth_image.astype(np.float32),cv2.COLOR_BGR2GRAY) / 255
##empty_depth_image_gray = np.load('./depthempty.npy')
rgb_image = cv2.imread("rgb_image15.png")
depth_image = cv2.imread("depth_image15.png",cv2.IMREAD_UNCHANGED)
depth_image_gray = cv2.cvtColor(depth_image.astype(np.float32),cv2.COLOR_BGR2GRAY) / 255
#depth_image_gray = np.load('./depth.npy')

#zpos = 0.4#0.25
#depth_image_gray = depth_image_gray*zpos

#num_pixels,depth_count,total_count,valid_pixel_percentage = count(rgb_image,depth_image_gray)
num_pixels,depth_count,total_count,sum,valid_pixel_percentage = compare_count(empty_rgb_image,empty_depth_image_gray,rgb_image,depth_image_gray)
print(depth_count); print(sum)

#print('max val', np.max(depth_image))
# depth_image_gray = np.uint16(depth_image * (65535 / 255))
# print('max val', np.max(depth_image_gray))
# depth_image_gray = (depth_image_gray/65535)*0.25
#print(num_pixels); print(depth_count); print(total_count)

unique = count_unique(empty_depth_image_gray)
#print(unique)
#print(np.min(depth_image_gray))

unique = count_unique(rgb_image)
#print(unique)
