from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_WIDTH = 0.42
BODY_DEPTH = 0.56
BODY_HEIGHT = 0.53
CASTER_HEIGHT = 0.06

SIDE_THICKNESS = 0.018
TOP_THICKNESS = 0.022
BOTTOM_THICKNESS = 0.018
BACK_THICKNESS = 0.012
DRAWER_REVEAL = 0.004

DRAWER_FRONT_THICKNESS = 0.018
DRAWER_FRONT_HEIGHT = (
    BODY_HEIGHT - TOP_THICKNESS - BOTTOM_THICKNESS - (2.0 * DRAWER_REVEAL)
) / 3.0
DRAWER_BOX_WIDTH = 0.33
DRAWER_BOX_DEPTH = 0.46
DRAWER_BOX_HEIGHT = DRAWER_FRONT_HEIGHT - 0.012
DRAWER_SIDE_THICKNESS = 0.012
DRAWER_BOTTOM_THICKNESS = 0.01
DRAWER_BACK_THICKNESS = 0.012
DRAWER_TRAVEL = 0.40

OUTER_RAIL_THICKNESS = 0.008
OUTER_RAIL_HEIGHT = 0.034
OUTER_RAIL_LENGTH = 0.50

INNER_RAIL_THICKNESS = 0.008
INNER_RAIL_HEIGHT = 0.032
INNER_RAIL_LENGTH = DRAWER_BOX_DEPTH

DRAWER_OPENING_Z_MIN = CASTER_HEIGHT + BOTTOM_THICKNESS
DRAWER_SLOT_PITCH = DRAWER_FRONT_HEIGHT + DRAWER_REVEAL

DRAWER_SPECS = (
    (
        "bottom",
        "bottom_drawer",
        "cabinet_to_bottom_drawer",
        DRAWER_OPENING_Z_MIN + (DRAWER_FRONT_HEIGHT / 2.0),
    ),
    (
        "middle",
        "middle_drawer",
        "cabinet_to_middle_drawer",
        DRAWER_OPENING_Z_MIN + (DRAWER_FRONT_HEIGHT / 2.0) + DRAWER_SLOT_PITCH,
    ),
    (
        "top",
        "top_drawer",
        "cabinet_to_top_drawer",
        DRAWER_OPENING_Z_MIN + (DRAWER_FRONT_HEIGHT / 2.0) + (2.0 * DRAWER_SLOT_PITCH),
    ),
)

CASTER_SPECS = (
    ("front_left_caster", -0.155, -0.215, 0.35),
    ("front_right_caster", 0.155, -0.215, -0.25),
    ("rear_left_caster", -0.155, 0.215, 1.10),
    ("rear_right_caster", 0.155, 0.215, -0.85),
)


def _add_drawer(
    model: ArticulatedObject,
    cabinet,
    *,
    slot_name: str,
    part_name: str,
    joint_name: str,
    z_center: float,
    face_material,
    interior_material,
    rail_material,
    handle_material,
) -> None:
    drawer = model.part(part_name)

    front_width = BODY_WIDTH - (2.0 * DRAWER_REVEAL)
    side_x = (DRAWER_BOX_WIDTH / 2.0) - (DRAWER_SIDE_THICKNESS / 2.0)
    inner_rail_x = (DRAWER_BOX_WIDTH / 2.0) + (INNER_RAIL_THICKNESS / 2.0)
    handle_z = 0.026

    drawer.visual(
        Box((front_width, DRAWER_FRONT_THICKNESS, DRAWER_FRONT_HEIGHT)),
        origin=Origin(xyz=(0.0, -(DRAWER_FRONT_THICKNESS / 2.0), 0.0)),
        material=face_material,
        name="drawer_front",
    )
    drawer.visual(
        Box((DRAWER_SIDE_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_BOX_HEIGHT)),
        origin=Origin(xyz=(-side_x, DRAWER_BOX_DEPTH / 2.0, 0.0)),
        material=interior_material,
        name="left_side",
    )
    drawer.visual(
        Box((DRAWER_SIDE_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_BOX_HEIGHT)),
        origin=Origin(xyz=(side_x, DRAWER_BOX_DEPTH / 2.0, 0.0)),
        material=interior_material,
        name="right_side",
    )
    drawer.visual(
        Box(
            (
                DRAWER_BOX_WIDTH - (2.0 * DRAWER_SIDE_THICKNESS),
                DRAWER_BACK_THICKNESS,
                DRAWER_BOX_HEIGHT,
            )
        ),
        origin=Origin(
            xyz=(0.0, DRAWER_BOX_DEPTH - (DRAWER_BACK_THICKNESS / 2.0), 0.0)
        ),
        material=interior_material,
        name="back_panel",
    )
    drawer.visual(
        Box(
            (
                DRAWER_BOX_WIDTH - (2.0 * DRAWER_SIDE_THICKNESS),
                DRAWER_BOX_DEPTH - DRAWER_BACK_THICKNESS,
                DRAWER_BOTTOM_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                (DRAWER_BOX_DEPTH - DRAWER_BACK_THICKNESS) / 2.0,
                -(DRAWER_BOX_HEIGHT / 2.0) + (DRAWER_BOTTOM_THICKNESS / 2.0),
            )
        ),
        material=interior_material,
        name="bottom_panel",
    )
    drawer.visual(
        Box((INNER_RAIL_THICKNESS, INNER_RAIL_LENGTH, INNER_RAIL_HEIGHT)),
        origin=Origin(xyz=(-inner_rail_x, INNER_RAIL_LENGTH / 2.0, 0.0)),
        material=rail_material,
        name="left_inner_rail",
    )
    drawer.visual(
        Box((INNER_RAIL_THICKNESS, INNER_RAIL_LENGTH, INNER_RAIL_HEIGHT)),
        origin=Origin(xyz=(inner_rail_x, INNER_RAIL_LENGTH / 2.0, 0.0)),
        material=rail_material,
        name="right_inner_rail",
    )
    drawer.visual(
        Box((0.012, 0.01, 0.018)),
        origin=Origin(xyz=(-0.05, -0.023, handle_z)),
        material=handle_material,
        name="left_handle_post",
    )
    drawer.visual(
        Box((0.012, 0.01, 0.018)),
        origin=Origin(xyz=(0.05, -0.023, handle_z)),
        material=handle_material,
        name="right_handle_post",
    )
    drawer.visual(
        Box((0.132, 0.016, 0.01)),
        origin=Origin(xyz=(0.0, -0.036, handle_z)),
        material=handle_material,
        name="handle_bar",
    )

    model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(0.0, -(BODY_DEPTH / 2.0), z_center)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
        meta={"slot": slot_name},
    )


def _add_caster(
    model: ArticulatedObject,
    cabinet,
    *,
    part_name: str,
    x: float,
    y: float,
    swivel_yaw: float,
    metal_material,
    wheel_material,
) -> None:
    caster = model.part(part_name)

    caster.visual(
        Box((0.05, 0.05, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=metal_material,
        name="mount_plate",
    )
    caster.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=metal_material,
        name="swivel_stem",
    )
    caster.visual(
        Box((0.026, 0.02, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        material=metal_material,
        name="yoke_block",
    )
    caster.visual(
        Box((0.005, 0.02, 0.022)),
        origin=Origin(xyz=(-0.01, 0.0, -0.035)),
        material=metal_material,
        name="left_fork",
    )
    caster.visual(
        Box((0.005, 0.02, 0.022)),
        origin=Origin(xyz=(0.01, 0.0, -0.035)),
        material=metal_material,
        name="right_fork",
    )
    caster.visual(
        Cylinder(radius=0.0035, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.04), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal_material,
        name="axle",
    )
    caster.visual(
        Cylinder(radius=0.02, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.04), rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_material,
        name="wheel",
    )

    model.articulation(
        f"cabinet_to_{part_name}",
        ArticulationType.FIXED,
        parent=cabinet,
        child=caster,
        origin=Origin(xyz=(x, y, CASTER_HEIGHT), rpy=(0.0, 0.0, swivel_yaw)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_desk_mobile_pedestal")

    body_paint = model.material("body_paint", rgba=(0.87, 0.88, 0.89, 1.0))
    drawer_face = model.material("drawer_face", rgba=(0.84, 0.85, 0.86, 1.0))
    drawer_interior = model.material("drawer_interior", rgba=(0.94, 0.94, 0.95, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_handle = model.material("dark_handle", rgba=(0.27, 0.29, 0.31, 1.0))
    caster_metal = model.material("caster_metal", rgba=(0.69, 0.71, 0.74, 1.0))
    caster_wheel = model.material("caster_wheel", rgba=(0.16, 0.16, 0.17, 1.0))

    cabinet = model.part("cabinet")

    cabinet.visual(
        Box((SIDE_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                -((BODY_WIDTH / 2.0) - (SIDE_THICKNESS / 2.0)),
                0.0,
                CASTER_HEIGHT + (BODY_HEIGHT / 2.0),
            )
        ),
        material=body_paint,
        name="left_side_panel",
    )
    cabinet.visual(
        Box((SIDE_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                (BODY_WIDTH / 2.0) - (SIDE_THICKNESS / 2.0),
                0.0,
                CASTER_HEIGHT + (BODY_HEIGHT / 2.0),
            )
        ),
        material=body_paint,
        name="right_side_panel",
    )
    cabinet.visual(
        Box((BODY_WIDTH, BODY_DEPTH, TOP_THICKNESS)),
        origin=Origin(
            xyz=(0.0, 0.0, CASTER_HEIGHT + BODY_HEIGHT - (TOP_THICKNESS / 2.0))
        ),
        material=body_paint,
        name="top_panel",
    )
    cabinet.visual(
        Box((BODY_WIDTH, BODY_DEPTH, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CASTER_HEIGHT + (BOTTOM_THICKNESS / 2.0))),
        material=body_paint,
        name="bottom_panel",
    )
    cabinet.visual(
        Box(
            (
                BODY_WIDTH - (2.0 * SIDE_THICKNESS),
                BACK_THICKNESS,
                BODY_HEIGHT - TOP_THICKNESS - BOTTOM_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                (BODY_DEPTH / 2.0) - (BACK_THICKNESS / 2.0),
                CASTER_HEIGHT
                + BOTTOM_THICKNESS
                + (BODY_HEIGHT - TOP_THICKNESS - BOTTOM_THICKNESS) / 2.0,
            )
        ),
        material=body_paint,
        name="back_panel",
    )

    outer_rail_x = (BODY_WIDTH / 2.0) - SIDE_THICKNESS - (OUTER_RAIL_THICKNESS / 2.0)
    outer_rail_center_y = -(BODY_DEPTH / 2.0) + 0.01 + (OUTER_RAIL_LENGTH / 2.0)
    for slot_name, _, _, z_center in DRAWER_SPECS:
        cabinet.visual(
            Box((OUTER_RAIL_THICKNESS, OUTER_RAIL_LENGTH, OUTER_RAIL_HEIGHT)),
            origin=Origin(xyz=(-outer_rail_x, outer_rail_center_y, z_center)),
            material=rail_steel,
            name=f"{slot_name}_left_outer_rail",
        )
        cabinet.visual(
            Box((OUTER_RAIL_THICKNESS, OUTER_RAIL_LENGTH, OUTER_RAIL_HEIGHT)),
            origin=Origin(xyz=(outer_rail_x, outer_rail_center_y, z_center)),
            material=rail_steel,
            name=f"{slot_name}_right_outer_rail",
        )

    for slot_name, part_name, joint_name, z_center in DRAWER_SPECS:
        _add_drawer(
            model,
            cabinet,
            slot_name=slot_name,
            part_name=part_name,
            joint_name=joint_name,
            z_center=z_center,
            face_material=drawer_face,
            interior_material=drawer_interior,
            rail_material=rail_steel,
            handle_material=dark_handle,
        )

    for part_name, x, y, swivel_yaw in CASTER_SPECS:
        _add_caster(
            model,
            cabinet,
            part_name=part_name,
            x=x,
            y=y,
            swivel_yaw=swivel_yaw,
            metal_material=caster_metal,
            wheel_material=caster_wheel,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    bottom_drawer = object_model.get_part("bottom_drawer")
    middle_drawer = object_model.get_part("middle_drawer")
    top_drawer = object_model.get_part("top_drawer")

    drawer_parts = {
        "bottom": bottom_drawer,
        "middle": middle_drawer,
        "top": top_drawer,
    }
    drawer_joints = {
        "bottom": object_model.get_articulation("cabinet_to_bottom_drawer"),
        "middle": object_model.get_articulation("cabinet_to_middle_drawer"),
        "top": object_model.get_articulation("cabinet_to_top_drawer"),
    }

    for caster_name, _, _, _ in CASTER_SPECS:
        caster = object_model.get_part(caster_name)
        ctx.expect_contact(
            caster,
            cabinet,
            name=f"{caster_name} mounts to the cabinet underside",
        )

    ctx.expect_gap(
        middle_drawer,
        bottom_drawer,
        axis="z",
        positive_elem="drawer_front",
        negative_elem="drawer_front",
        min_gap=DRAWER_REVEAL - 0.0005,
        max_gap=DRAWER_REVEAL + 0.0005,
        name="middle drawer reveal matches the lower reveal",
    )
    ctx.expect_gap(
        top_drawer,
        middle_drawer,
        axis="z",
        positive_elem="drawer_front",
        negative_elem="drawer_front",
        min_gap=DRAWER_REVEAL - 0.0005,
        max_gap=DRAWER_REVEAL + 0.0005,
        name="top drawer reveal matches the middle reveal",
    )

    front_heights = []
    for drawer in (bottom_drawer, middle_drawer, top_drawer):
        aabb = ctx.part_element_world_aabb(drawer, elem="drawer_front")
        if aabb is not None:
            front_heights.append(aabb[1][2] - aabb[0][2])
    ctx.check(
        "drawer fronts are equal height",
        len(front_heights) == 3 and max(front_heights) - min(front_heights) <= 1e-6,
        details=f"front_heights={front_heights}",
    )

    for slot_name, _, _, _ in DRAWER_SPECS:
        drawer = drawer_parts[slot_name]
        slide = drawer_joints[slot_name]

        ctx.expect_within(
            drawer,
            cabinet,
            axes="xz",
            inner_elem="drawer_front",
            name=f"{slot_name} drawer front stays aligned to the cabinet opening",
        )
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="y",
            elem_a="left_inner_rail",
            elem_b=f"{slot_name}_left_outer_rail",
            min_overlap=0.45,
            name=f"{slot_name} left rail stays deeply engaged when closed",
        )
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="y",
            elem_a="right_inner_rail",
            elem_b=f"{slot_name}_right_outer_rail",
            min_overlap=0.45,
            name=f"{slot_name} right rail stays deeply engaged when closed",
        )

        rest_position = ctx.part_world_position(drawer)
        upper = slide.motion_limits.upper if slide.motion_limits is not None else None
        with ctx.pose({slide: upper if upper is not None else DRAWER_TRAVEL}):
            ctx.expect_within(
                drawer,
                cabinet,
                axes="xz",
                inner_elem="drawer_front",
                name=f"{slot_name} drawer front stays laterally aligned when extended",
            )
            ctx.expect_overlap(
                drawer,
                cabinet,
                axes="y",
                min_overlap=0.048,
                elem_a="left_inner_rail",
                elem_b=f"{slot_name}_left_outer_rail",
                name=f"{slot_name} left rail retains insertion at full extension",
            )
            ctx.expect_overlap(
                drawer,
                cabinet,
                axes="y",
                min_overlap=0.048,
                elem_a="right_inner_rail",
                elem_b=f"{slot_name}_right_outer_rail",
                name=f"{slot_name} right rail retains insertion at full extension",
            )
            extended_position = ctx.part_world_position(drawer)

        ctx.check(
            f"{slot_name} drawer extends outward",
            rest_position is not None
            and extended_position is not None
            and extended_position[1] < rest_position[1] - 0.35,
            details=f"rest={rest_position}, extended={extended_position}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
