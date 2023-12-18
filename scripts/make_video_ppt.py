import pickle
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw, ImageFont
import os
import imageio
from tqdm import trange
import numpy as np

if __name__ == "__main__":
    info = pickle.load(open('./media/info3.pickle', 'rb'))
    frames = []

    T = info['observed_wrench'].shape[0]
    max_ft = np.max(info['observed_wrench'])
    min_ft = np.min(info['observed_wrench'])

    height = info['left_cap2'][0].shape[0]
    width = info['left_cap2'][0].shape[1]

    for i in trange(len(info['left_cap2'])):
        # tp_idx = int(i/12)
        wrist_indx = i*12
        third_person_img = Image.fromarray(info['left_cap2'][i])
        turret_img = Image.fromarray(info['eye_in_hand_90'][i])
        wrist_torque = info['observed_wrench'][:wrist_indx+1, :]

        frame = Image.new('RGB', (width+int(width/2), height), color='white')

        # Paste images from list1 and list2
        frame.paste(third_person_img, (0, 0))
        frame.paste(turret_img, (width, 0))

        # Create a subplot for the array plot
        # plt.figure(figsize=(5.14, 5.14))
        plt.figure(figsize=(width/200, height/200))
        # plt.axis('off')
        plt.plot(wrist_torque[:,0], linewidth=1.5)
        plt.plot(wrist_torque[:,1], linewidth=1.5)
        plt.plot(wrist_torque[:,2], linewidth=1.5)
        plt.xlim(0, T)
        plt.ylim(min_ft, max_ft)
        plt.ylabel("FT")
        plt.xlabel("time")
        plt.xticks([])
        plt.yticks([])
        plt.scatter(wrist_indx, wrist_torque[wrist_indx,0])
        plt.scatter(wrist_indx, wrist_torque[wrist_indx,1])
        plt.scatter(wrist_indx, wrist_torque[wrist_indx,2])

        plt.savefig('./media/temp_plot.png', bbox_inches='tight', pad_inches=0, transparent=False)
        plt.close()
        
        # Paste the array plot onto the frame
        plot_img = Image.open('./media/temp_plot.png')
        frame.paste(plot_img, (width, int(height/2)))
        # import ipdb; ipdb.set_trace()
        frames.append(frame)
        os.remove('./media/temp_plot.png')

    # frames[0].save('./media/output.gif', save_all=True, append_images=frames[1:], duration=100, loop=0)

    # Video saving
    # cl = ImageSequenceClip(vid_static, fps=10)
    # cl.write_gif('./media/vid_static_sideview2.gif', fps=10, logger=None)
    # del cl
    imageio.mimsave('./media/ballbot_pickup_multimodal_480_640.gif', frames, duration=0.01)
