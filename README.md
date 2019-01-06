# Mesh_registration

This is my work on the mesh registration problem

## Getting Started

### Prerequisite

Matlab with the Statistics Toolbox

### Installing
Change in config.m the path to the data.
The data must be in an .OBJ format.
I used Blender to transformed the .ply to .obj

### Running the test

Run the demo.m file.

## Implementation and Result

A the beginning, the two face are not oriented the same way and the expressions are different. After running the ICP algorithm, the 2 faces are closely aligned.

![Init_state](https://github.com/AlexTintin/Mesh_registration/blob/master/images/Init_state.PNG)
![Final_state](https://github.com/AlexTintin/Mesh_registration/blob/master/images/Final_state.PNG)

The RMS error has greatly decreases but there seem to be a small rotational difference between the two faces.
The expression on the neutral face has not changed.

![Final_state2](https://github.com/AlexTintin/Mesh_registration/blob/master/images/Final_state2.PNG)
![Error](https://github.com/AlexTintin/Mesh_registration/blob/master/images/Error.PNG)

A brief synthesis of my work as well as an analisys of the results is avalale here [Results](https://drive.google.com/file/d/1NRmjS41s9Bvgez30hoMaTTYAvF0UNnJ5/view?usp=sharing)

## Acknowledgment

This code uses a part of the work of Martin Kjer and Jakob Wilm from the Technical University of Denmark. Their work can be found [here](https://fr.mathworks.com/matlabcentral/fileexchange/27804-iterative-closest-point)
```
@mastersthesis{kjer2010evaluation,
  title={Evaluation of surface registration algorithms for PET motion correction},
  author={Kjer, Hans Martin and Wilm, Jakob},
  type={{B.S.} thesis},
  year={2010},
  school={Technical University of Denmark, DTU, DK-2800 Kgs. Lyngby, Denmark}
}
```