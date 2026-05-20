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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, *, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _annular_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    segments: int = 64,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=segments),
            [_circle_profile(inner_radius, segments=segments)],
            height=thickness,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_laundromat_tumble_dryer")

    body_width = 1.02
    body_depth = 1.04
    body_height = 1.16
    wall = 0.03
    front_frame_t = 0.035
    front_y = body_depth * 0.5
    door_center_z = 0.62
    door_outer_radius = 0.36
    door_inner_radius = 0.24
    door_thickness = 0.055
    door_hinge_x = -door_outer_radius
    door_hinge_y = front_y + door_thickness * 0.5 + 0.010
    drum_outer_radius = 0.325
    drum_inner_radius = 0.302
    drum_length = 0.72
    drum_axis_y = 0.06

    cabinet_white = model.material("cabinet_white", rgba=(0.94, 0.95, 0.96, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.27, 0.29, 0.31, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.09, 0.10, 0.11, 1.0))
    drum_steel = model.material("drum_steel", rgba=(0.64, 0.67, 0.70, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.38, 0.51, 0.58, 0.32))
    handle_black = model.material("handle_black", rgba=(0.12, 0.13, 0.14, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((wall, body_depth, body_height)),
        origin=Origin(xyz=(-body_width * 0.5 + wall * 0.5, 0.0, body_height * 0.5)),
        material=cabinet_white,
        name="left_side",
    )
    cabinet.visual(
        Box((wall, body_depth, body_height)),
        origin=Origin(xyz=(body_width * 0.5 - wall * 0.5, 0.0, body_height * 0.5)),
        material=cabinet_white,
        name="right_side",
    )
    cabinet.visual(
        Box((body_width - 2.0 * wall, body_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall * 0.5)),
        material=cabinet_white,
        name="base_pan",
    )
    cabinet.visual(
        Box((body_width - 2.0 * wall, body_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_height - wall * 0.5)),
        material=cabinet_white,
        name="top_panel",
    )
    cabinet.visual(
        Box((body_width - 2.0 * wall, wall, body_height - 2.0 * wall)),
        origin=Origin(xyz=(0.0, -body_depth * 0.5 + wall * 0.5, body_height * 0.5)),
        material=cabinet_white,
        name="rear_panel",
    )

    opening_width = 0.84
    opening_height = 0.88
    side_stile = (body_width - opening_width) * 0.5
    bottom_rail = door_center_z - opening_height * 0.5
    top_rail = body_height - (door_center_z + opening_height * 0.5)

    cabinet.visual(
        Box((side_stile, front_frame_t, body_height)),
        origin=Origin(
            xyz=(-body_width * 0.5 + side_stile * 0.5, front_y - front_frame_t * 0.5, body_height * 0.5)
        ),
        material=cabinet_white,
        name="front_left_stile",
    )
    cabinet.visual(
        Box((side_stile, front_frame_t, body_height)),
        origin=Origin(
            xyz=(body_width * 0.5 - side_stile * 0.5, front_y - front_frame_t * 0.5, body_height * 0.5)
        ),
        material=cabinet_white,
        name="front_right_stile",
    )
    cabinet.visual(
        Box((opening_width, front_frame_t, bottom_rail)),
        origin=Origin(xyz=(0.0, front_y - front_frame_t * 0.5, bottom_rail * 0.5)),
        material=cabinet_white,
        name="front_bottom_rail",
    )
    cabinet.visual(
        Box((opening_width, front_frame_t, top_rail)),
        origin=Origin(
            xyz=(
                0.0,
                front_y - front_frame_t * 0.5,
                door_center_z + opening_height * 0.5 + top_rail * 0.5,
            )
        ),
        material=cabinet_white,
        name="front_top_rail",
    )
    cabinet.visual(
        Box((body_width - 2.0 * wall, 0.05, 0.14)),
        origin=Origin(xyz=(0.0, front_y - 0.025, body_height - 0.07)),
        material=trim_gray,
        name="control_fascia",
    )

    front_collar = _annular_mesh(
        "dryer_front_collar",
        outer_radius=0.39,
        inner_radius=0.29,
        thickness=0.05,
    )
    cabinet.visual(
        front_collar,
        origin=Origin(
            xyz=(0.0, front_y - front_frame_t - 0.025, door_center_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_gray,
        name="front_collar",
    )
    cabinet.visual(
        Box((0.06, 0.05, 0.09)),
        origin=Origin(xyz=(-0.40, front_y - front_frame_t - 0.025, door_center_z)),
        material=dark_gray,
        name="collar_left_bridge",
    )
    cabinet.visual(
        Box((0.06, 0.05, 0.09)),
        origin=Origin(xyz=(0.40, front_y - front_frame_t - 0.025, door_center_z)),
        material=dark_gray,
        name="collar_right_bridge",
    )
    cabinet.visual(
        Box((0.10, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, front_y - front_frame_t - 0.025, door_center_z + 0.40)),
        material=dark_gray,
        name="collar_top_bridge",
    )
    cabinet.visual(
        Box((0.10, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, front_y - front_frame_t - 0.025, door_center_z - 0.40)),
        material=dark_gray,
        name="collar_bottom_bridge",
    )

    for z_pos, name in ((door_center_z + 0.22, "upper"), (door_center_z - 0.22, "lower")):
        cabinet.visual(
            Box((0.10, 0.04, 0.12)),
            origin=Origin(xyz=(door_hinge_x - 0.03, front_y, z_pos)),
            material=dark_gray,
            name=f"hinge_bracket_{name}",
        )
    cabinet.visual(
        Cylinder(radius=0.050, length=0.13),
        origin=Origin(
            xyz=(0.0, drum_axis_y - drum_length * 0.5 - 0.065, door_center_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_gray,
        name="rear_bearing_carrier",
    )
    cabinet.visual(
        Box((0.18, 0.10, 0.16)),
        origin=Origin(xyz=(0.0, -body_depth * 0.5 + 0.05, door_center_z)),
        material=dark_gray,
        name="rear_bearing_housing",
    )

    cabinet.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=160.0,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    door = model.part("door")
    door_ring = _annular_mesh(
        "dryer_door_ring",
        outer_radius=door_outer_radius,
        inner_radius=door_inner_radius,
        thickness=door_thickness,
    )
    door.visual(
        door_ring,
        origin=Origin(xyz=(door_outer_radius, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="door_ring",
    )
    door.visual(
        Cylinder(radius=0.240, length=0.008),
        origin=Origin(xyz=(door_outer_radius, 0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass_tint,
        name="door_glass",
    )
    door.visual(
        Box((0.045, 0.030, 0.62)),
        origin=Origin(xyz=(0.0225, 0.0, 0.0)),
        material=dark_gray,
        name="hinge_rail",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=dark_gray,
        name="upper_knuckle",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, -0.22)),
        material=dark_gray,
        name="lower_knuckle",
    )
    door.visual(
        Box((0.060, 0.038, 0.18)),
        origin=Origin(xyz=(0.69, 0.042, 0.0)),
        material=handle_black,
        name="door_pull",
    )
    door.visual(
        _annular_mesh(
            "dryer_inner_gasket",
            outer_radius=0.240,
            inner_radius=0.228,
            thickness=0.018,
        ),
        origin=Origin(xyz=(door_outer_radius, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gasket_black,
        name="inner_gasket",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.78, 0.10, 0.82)),
        mass=18.0,
        origin=Origin(xyz=(0.39, 0.0, 0.0)),
    )

    drum = model.part("drum")
    drum_shell = _annular_mesh(
        "dryer_drum_shell",
        outer_radius=drum_outer_radius,
        inner_radius=drum_inner_radius,
        thickness=drum_length,
    )
    drum.visual(
        drum_shell,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=drum_steel,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=drum_inner_radius, length=0.012),
        origin=Origin(
            xyz=(0.0, -drum_length * 0.5 + 0.006, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=drum_steel,
        name="rear_bulkhead",
    )
    drum.visual(
        Cylinder(radius=0.055, length=0.05),
        origin=Origin(
            xyz=(0.0, -drum_length * 0.5 + 0.025, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_gray,
        name="rear_hub",
    )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=drum_outer_radius, length=drum_length),
        mass=28.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(door_hinge_x, door_hinge_y, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.8,
            lower=0.0,
            upper=2.05,
        ),
    )
    model.articulation(
        "cabinet_to_drum",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, drum_axis_y, door_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=9.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    drum = object_model.get_part("drum")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    drum_spin = object_model.get_articulation("cabinet_to_drum")

    ctx.check(
        "door uses a vertical revolute hinge at the left edge",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and door_hinge.axis == (0.0, 0.0, 1.0)
        and door_hinge.origin.xyz[0] < 0.0,
        details=f"type={door_hinge.articulation_type}, axis={door_hinge.axis}, origin={door_hinge.origin.xyz}",
    )
    ctx.check(
        "drum uses a horizontal continuous rotation axis",
        drum_spin.articulation_type == ArticulationType.CONTINUOUS
        and drum_spin.axis == (0.0, 1.0, 0.0),
        details=f"type={drum_spin.articulation_type}, axis={drum_spin.axis}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            min_overlap=0.70,
            name="closed door spans the dryer front opening",
        )
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            min_gap=0.006,
            max_gap=0.030,
            positive_elem="door_ring",
            negative_elem="front_bottom_rail",
            name="closed door sits just proud of the front face",
        )
        ctx.expect_within(
            drum,
            cabinet,
            axes="xz",
            margin=0.0,
            name="drum stays contained within the cabinet envelope",
        )

    rest_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.20}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward when opened",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > rest_aabb[1][1] + 0.18,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
