from yourdfpy import URDF, Robot, Link, Visual, Geometry, Box, Material, Color

red = Material(name="red", color=Color(rgba=[1.0, 0.0, 0.0, 0.5]))

robot = Robot(
    name="box",
    materials=[red],
    links=[
        Link(
            name="base",
            visuals=[
                Visual(
                    geometry=Geometry(box=Box(size=[0.2, 0.2, 0.2])),
                    material=red,
                )
            ],
        )
    ],
)


fallback_urdf_model = URDF(robot=robot)
