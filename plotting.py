import matplotlib.pyplot as plt
import matplotlib.patches as patches

import env


class Plotting:
    def __init__(self, x_start, x_goal):
        self.xI, self.xG = x_start, x_goal
        self.env = env.Env()
        self.obs_bound = self.env.obs_boundary
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle

    def animation(self, nodelist, path, path_convert,circles,output,name, animation=False):
        self.plot_grid(name,circles)
        self.plot_visited(nodelist, animation)
        self.plot_path(path,path_convert,output)
        

    def animation_connect(self, V1, V2, path,path_convert,circles,output, name):
        self.plot_grid(name,circles)
        self.plot_visited_connect(V1, V2)
        plot_path(path,path_convert,output)
        


    def plot_grid(self, name,circles):
        fig, ax = plt.subplots()

        if self.obs_bound is not None:
            for (ox, oy, w, h) in self.obs_bound:
                ax.add_patch(
                    patches.Rectangle(
                        (ox, oy), w, h,
                        edgecolor='black',
                        facecolor='black',
                        fill=True
                    )
                )

        if self.obs_rectangle is not None:
            for (ox, oy, w, h) in self.obs_rectangle:
                ax.add_patch(
                    patches.Rectangle(
                        (ox, oy), w, h,
                        edgecolor='black',
                        facecolor='black',
                        fill=True
                    )
                )

        if self.obs_circle is not None:
            for (ox, oy, r) in self.obs_circle:
                ax.add_patch(
                    patches.Circle(
                        (ox, oy), r,
                        edgecolor='black',
                        facecolor='black',
                        fill=True
                    )
                )
        if circles is not None:
            for (ox, oy, r) in circles:
                ax.add_patch(
                patches.Circle(
                        (ox, oy), r,
                        edgecolor='red',
                        linestyle='dashed',
                        facecolor='white',
                        fill=True
                    )
                )
        

        plt.plot(self.xI[0], self.xI[1], "bs", linewidth=3)
        plt.plot(self.xG[0], self.xG[1], "gs", linewidth=3)

        plt.title(name)
        plt.axis("equal")

    @staticmethod
    def plot_visited(nodelist, animation):
        if animation:
            count = 0
            for node in nodelist:
                count += 1
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:
                        plt.pause(0.001)
        else:
            for node in nodelist:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    @staticmethod
    def plot_visited_connect(V1, V2):
        len1, len2 = len(V1), len(V2)

        for k in range(max(len1, len2)):
            if k < len1:
                if V1[k].parent:
                    plt.plot([V1[k].x, V1[k].parent.x], [V1[k].y, V1[k].parent.y], "-g")
            if k < len2:
                if V2[k].parent:
                    plt.plot([V2[k].x, V2[k].parent.x], [V2[k].y, V2[k].parent.y], "-g")

            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])
            if k % 2 == 0:
                plt.pause(0.001)

        plt.pause(0.01)
    # @staticmethod
    # def plot_path_convert(path_convert):
    #     if len(path_convert) != 0:
    #         plt.plot([x[0] for x in path_convert], [x[1] for x in path_convert], '-r', linewidth=2)
    #         plt.pause(0.01)
    #     plt.show()
    @staticmethod
    def plot_path(path,path_convert,output):
        # fig1, ax1 = plt.subplots()
        # ax1.set(xlim=(0, 800), ylim = (0, 600))
        if len(path_convert) != 0:
            plt.plot([x[0] for x in path_convert], [x[1] for x in path_convert], '-r', linewidth=3)
            plt.pause(0.01)
        if len(path) != 0:
            plt.plot([x[0] for x in path], [x[1] for x in path], 'black', linewidth=2)
            plt.pause(0.01)
        if len(output) != 0:
            plt.plot([x[0] for x in output], [x[1] for x in output], 'blue', linewidth=2)
            plt.pause(0.01)
        plt.show()
       

        
    

