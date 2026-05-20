from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_vent_tower")

    galvanized = model.material("galvanized", rgba=(0.70, 0.72, 0.74, 1.0))
    flashing = model.material("flashing", rgba=(0.54, 0.56, 0.59, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.26, 0.27, 0.29, 1.0))

    tower = model.part("tower")

    tower.visual(
        Box((0.72, 0.58, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=flashing,
        name="roof_flashing",
    )
    tower.visual(
        Box((0.34, 0.018, 0.82)),
        origin=Origin(xyz=(0.0, -0.131, 0.422)),
        material=galvanized,
        name="rear_wall",
    )
    tower.visual(
        Box((0.018, 0.282, 0.82)),
        origin=Origin(xyz=(-0.161, 0.0, 0.422)),
        material=galvanized,
        name="left_wall",
    )
    tower.visual(
        Box((0.018, 0.282, 0.82)),
        origin=Origin(xyz=(0.161, 0.0, 0.422)),
        material=galvanized,
        name="right_wall",
    )
    tower.visual(
        Box((0.34, 0.018, 0.55)),
        origin=Origin(xyz=(0.0, 0.131, 0.275)),
        material=galvanized,
        name="front_lower_wall",
    )
    tower.visual(
        Box((0.05, 0.018, 0.22)),
        origin=Origin(xyz=(-0.145, 0.131, 0.66)),
        material=galvanized,
        name="front_left_jamb",
    )
    tower.visual(
        Box((0.05, 0.018, 0.22)),
        origin=Origin(xyz=(0.145, 0.131, 0.66)),
        material=galvanized,
        name="front_right_jamb",
    )
    tower.visual(
        Box((0.34, 0.018, 0.05)),
        origin=Origin(xyz=(0.0, 0.131, 0.795)),
        material=galvanized,
        name="front_header",
    )
    tower.visual(
        Box((0.37, 0.32, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.835)),
        material=galvanized,
        name="top_cap",
    )
    tower.visual(
        Box((0.32, 0.026, 0.034)),
        origin=Origin(xyz=(0.0, 0.145, 0.553)),
        material=flashing,
        name="outlet_frame_sill",
    )
    tower.visual(
        Box((0.32, 0.026, 0.034)),
        origin=Origin(xyz=(0.0, 0.145, 0.777)),
        material=flashing,
        name="outlet_frame_header",
    )
    tower.visual(
        Box((0.038, 0.026, 0.242)),
        origin=Origin(xyz=(-0.139, 0.145, 0.665)),
        material=flashing,
        name="outlet_frame_left",
    )
    tower.visual(
        Box((0.038, 0.026, 0.242)),
        origin=Origin(xyz=(0.139, 0.145, 0.665)),
        material=flashing,
        name="outlet_frame_right",
    )
    tower.visual(
        Box((0.036, 0.014, 0.028)),
        origin=Origin(xyz=(-0.11, 0.152, 0.763)),
        material=dark_metal,
        name="hinge_pad_left",
    )
    tower.visual(
        Box((0.036, 0.014, 0.028)),
        origin=Origin(xyz=(0.11, 0.152, 0.763)),
        material=dark_metal,
        name="hinge_pad_right",
    )
    tower.visual(
        Cylinder(radius=0.009, length=0.048),
        origin=Origin(xyz=(-0.088, 0.145, 0.777), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel_left",
    )
    tower.visual(
        Cylinder(radius=0.009, length=0.048),
        origin=Origin(xyz=(0.088, 0.145, 0.777), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel_right",
    )
    tower.inertial = Inertial.from_geometry(
        Box((0.72, 0.58, 0.85)),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
    )

    flap = model.part("weather_flap")
    flap.visual(
        Box((0.29, 0.01, 0.255)),
        origin=Origin(xyz=(0.0, 0.005, -0.1275)),
        material=galvanized,
        name="flap_panel",
    )
    flap.visual(
        Box((0.29, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, 0.008, -0.009)),
        material=galvanized,
        name="top_hem",
    )
    flap.visual(
        Box((0.018, 0.024, 0.22)),
        origin=Origin(xyz=(-0.136, 0.012, -0.121)),
        material=galvanized,
        name="left_edge_hem",
    )
    flap.visual(
        Box((0.018, 0.024, 0.22)),
        origin=Origin(xyz=(0.136, 0.012, -0.121)),
        material=galvanized,
        name="right_edge_hem",
    )
    flap.visual(
        Box((0.032, 0.012, 0.024)),
        origin=Origin(xyz=(-0.11, 0.006, -0.012)),
        material=dark_metal,
        name="hinge_tab_left",
    )
    flap.visual(
        Box((0.032, 0.012, 0.024)),
        origin=Origin(xyz=(0.11, 0.006, -0.012)),
        material=dark_metal,
        name="hinge_tab_right",
    )
    flap.inertial = Inertial.from_geometry(
        Box((0.31, 0.03, 0.27)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.012, -0.125)),
    )

    model.articulation(
        "tower_to_flap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=flap,
        origin=Origin(xyz=(0.0, 0.159, 0.777)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(68.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower")
    flap = object_model.get_part("weather_flap")
    hinge = object_model.get_articulation("tower_to_flap")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            flap,
            tower,
            axis="y",
            positive_elem="flap_panel",
            negative_elem="outlet_frame_header",
            min_gap=0.0005,
            max_gap=0.008,
            name="closed flap sits just off the outlet frame",
        )
        ctx.expect_overlap(
            flap,
            tower,
            axes="x",
            elem_a="flap_panel",
            elem_b="outlet_frame_header",
            min_overlap=0.25,
            name="closed flap spans the framed outlet width",
        )
        closed_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    with ctx.pose({hinge: math.radians(60.0)}):
        open_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    opened_outward = False
    opened_upward = False
    details = f"closed_aabb={closed_aabb}, open_aabb={open_aabb}"
    if closed_aabb is not None and open_aabb is not None:
        closed_center_y = (closed_aabb[0][1] + closed_aabb[1][1]) * 0.5
        open_center_y = (open_aabb[0][1] + open_aabb[1][1]) * 0.5
        closed_center_z = (closed_aabb[0][2] + closed_aabb[1][2]) * 0.5
        open_center_z = (open_aabb[0][2] + open_aabb[1][2]) * 0.5
        opened_outward = open_center_y > closed_center_y + 0.08
        opened_upward = open_center_z > closed_center_z + 0.04
        details = (
            f"closed_center_y={closed_center_y:.4f}, open_center_y={open_center_y:.4f}, "
            f"closed_center_z={closed_center_z:.4f}, open_center_z={open_center_z:.4f}"
        )

    ctx.check("flap opens outward", opened_outward, details=details)
    ctx.check("flap opens upward", opened_upward, details=details)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
