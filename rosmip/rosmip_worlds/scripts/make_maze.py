#!/usr/bin/env python

import sys, os, logging, argparse, math, numpy as np, cv2
import lxml.etree as ET

LOG = logging.getLogger('make_maze')
import pdb

class Maze:
    def __init__(self, **kwargs):
        
        if 'xml_path' in kwargs: self.read_xml(kwargs['xml_path'])
        else:
            self.walls = kwargs.get('walls', [])
            self.walls_height = kwargs.get('walls_height', 0.05)
            self.walls_width  = kwargs.get('walls_width', 0.01)

        all_points = np.array(self.walls).reshape((2*len(self.walls), 2))
        self.p_ld = np.min(all_points, axis=0)
        self.p_ru = np.max(all_points, axis=0)
        self.center = (self.p_ld+self.p_ru)/2
        self.extends =  self.p_ru - self.p_ld
        LOG.info("   extends {} center {}".format(self.extends, self.center))


    def read_xml(self, path):
        LOG.info(" reading maze from {}".format(path))
        self.walls = []
        tree = ET.ElementTree(file=path)
        xml_maze = tree.getroot()
        self.walls_height = float(xml_maze.attrib.get('walls_height', 0.05))
        self.walls_width = float(xml_maze.attrib.get('walls_width', 0.01))
        for e in xml_maze:
            if e.tag == 'wall':
                p1, p2 = [[float(f) for f in e.attrib[a].split(',')] for a in ['p0', 'p1']]
                self.walls.append([p1, p2])
        LOG.info("   found {} walls".format(len(self.walls)))  

class JulieDefaultMaze(Maze):
    def __init__(self):
        walls = [[( 1.,  1.), (29.,  1.)],
                 [( 1.,  1.), ( 1., 29.)],
                 [(29.,  1.), (29., 29.)],
                 [( 1., 29.), (29., 29.)],
                 [( 5., 10.), (29., 10.)],
                 [( 1., 15.), (24., 15.)]]
        Maze.__init__(self, walls=walls, walls_height=1., walls_width=0.1)


class RosMipDefaultMaze(Maze):
    def __init__(self):
        walls = [[( 1.,  1.), (3.,  1.)],
                 [( 1.,  1.), (1.,  3.)],
                 [( 3.,  1.), (3.,  3.)],
                 [( 1.,  3.), (3.,  3.)], 
                 [( 2.,  1.5), (2.,  2.5)], 
                 [( 2.,  1.5), (2.65, 1.5)],
                 [( 2.,  2.5), (2.65,  2.5)],
                 [( 2.35, 2.), (3.,  2.)]]
        Maze.__init__(self, walls=walls, walls_height=0.1, walls_width=0.01)




        
def make_gazebo_world(maze, path):
    LOG.info(" writting world to {}".format(path))
    xml_intro = '''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <ode>
        <solver>
          <!--type>quick</type>
          <dt>0.001</dt>
          <iters>40</iters>
          <sor>1.0</sor -->
          <!-- type>quick</type>
          <dt>0.01</dt>
          <iters>20</iters>
          <sor>1.0</sor -->
          <type>quick</type>
          <!--dt>0.001</dt-->
          <iters>20</iters>
          <sor>1.0</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.0</contact_surface_layer>
        </constraints>
      </ode>
      <max_step_size>0.001</max_step_size>
    </physics>
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
'''

    xml_outro='''
  </world>
</sdf>
'''
    with open(path, 'w') as f:
        f.write(xml_intro)
        for i, (p1, p2) in enumerate(maze.walls):
            f.write(get_wall_xml(i, np.array(p1), np.array(p2), maze.walls_height, maze.walls_width))
        f.write(xml_outro)  


def get_wall_xml(i, p1, p2, height, width):
    _v = p2-p1
    _len = np.linalg.norm(_v)
    #print _len
    box_size = "{}  {}  {}".format(_len, width, height)
    _center = (p1+p2)/2
    _angle = math.atan2(_v[1], _v[0])
    box_pose = "{:.1f} {:.1f} {:f} 0 0 {:f}".format(_center[0], _center[1], height/2, _angle)
    txt = '''
    <!-- A Wall -->
    <model name='wall_{:02d}'>
      <static>1</static>
      <link name='link'>
        <visual name='visual'>
          <geometry>
            <box>
              <size>{}</size>
            </box>
          </geometry>
        </visual>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <collision name='surface'>
          <pose frame=''>0 0 0 0 0 0</pose>
          <geometry>
            <box>
              <size>{}</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.6</mu>
                <mu2>0.6</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
      </link>
      <pose frame=''>{}</pose>
    </model>
'''.format(i, box_size, box_size, box_pose)
    return txt


def map_coor(p, center, map_resolution, map_size):
    dp = p - center
    x_img = dp[0]/map_resolution + map_size[1]/2
    y_img = map_size[0]/2 - dp[1]/map_resolution  -1
    return (int(x_img), int(y_img))
    
def map_origin(maze_center, map_resolution, map_size):
    return -0.5, -0.5


def make_map(maze, path, map_resolution=0.05, map_size=(600, 600, 1)):
    LOG.info(" writting map to {}.(yaml/png)".format(path))
    map_size = (int(math.ceil(maze.extends[1]/map_resolution))+3, int(math.ceil(maze.extends[0]/map_resolution))+3, 1)
    LOG.info("   map resolution {} map size {}x{}".format(map_resolution, map_size[1], map_size[0] ))
    img = np.zeros(map_size, np.uint8)
    cv2.rectangle(img, (0,0), (map_size[1], map_size[0]), (255), -1) # white background
    px_width = int(math.ceil(maze.walls_width/map_resolution))
    for p1, p2 in maze.walls:
        _p1, _p2 = [map_coor(np.array(p), maze.center, map_resolution, np.array(map_size)) for p in [p1, p2]]
        cv2.line(img, _p1, _p2, (0), px_width)
    
    img_path = path+'.png'
    cv2.imwrite(img_path, img)

    yaml_txt = '''
image: {}
resolution: {}
origin: [0., 0., 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
'''.format(os.path.basename(img_path), map_resolution)
    yaml_path = path+'.yaml'
    with open(yaml_path, 'w') as f:
        f.write(yaml_txt)


def parse_cmd_line():
    parser = argparse.ArgumentParser(description='Generate maps and gazebo worlds.')
    parser.add_argument('--maze_class', help='the class of maze to make', default=RosMipDefaultMaze)
    parser.add_argument('--maze_path', help='xml description of the maze', default=None)
    parser.add_argument('--map_resolution', help='the resolution of the map', default=0.01)
    parser.add_argument('--map_path', help='the output path for the generated map', default="/tmp/foo")
    parser.add_argument('--world_path', help='the output path for the generated world', default="/tmp/foo.world")
    return parser.parse_args()


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    args = parse_cmd_line()
    if args.maze_path is not None:
        maze = Maze(xml_path=args.maze_path)
    else:
        maze = args.maze_class()
    make_map(maze, args.map_path, float(args.map_resolution))
    make_gazebo_world(maze, args.world_path)

    #julie_dir = "/home/poine/work/simone/trunk/julie"
    #map_path = sys.argv[1] if len(sys.argv)>1 else os.path.join(julie_dir, "julie/julie_navigation/maps/maze4")
    #gazebo_world_path = sys.argv[2] if len(sys.argv)>2 else os.path.join(julie_dir, "julie_sim/julie_gazebo/worlds/maze4.world")
    #maze = Maze()
    #make_map(maze, map_path, map_resolution=0.025, map_size=(1200, 1200, 1))
    #make_gazebo_world(maze, gazebo_world_path)
