from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi, sqrt

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


TUBE_RADIUS = 0.012
RAIL_RADIUS = 0.0045
HINGE_BARREL_RADIUS = 0.010
DECK_HALF_WIDTH = 0.34
DECK_HALF_DEPTH = 0.25
DECK_HEIGHT = 0.88
HINGE_OFFSET = 0.046
WING_WIDTH = 0.46
WING_HALF_DEPTH = 0.25


def _add_rod_x(part, name: str, x0: float, x1: float, y: float, z: float, radius: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=abs(x1 - x0)),
        origin=Origin(xyz=((x0 + x1) * 0.5, y, z), rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_rod_y(part, name: str, x: float, y0: float, y1: float, z: float, radius: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=abs(y1 - y0)),
        origin=Origin(xyz=(x, (y0 + y1) * 0.5, z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_rod_z(part, name: str, x: float, y: float, z0: float, z1: float, radius: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=abs(z1 - z0)),
        origin=Origin(xyz=(x, y, (z0 + z1) * 0.5)),
        material=material,
        name=name,
    )


def _add_leg_yz(
    part,
    name: str,
    x: float,
    y0: float,
    z0: float,
    y1: float,
    z1: float,
    radius: float,
    material,
) -> None:
    dy = y1 - y0
    dz = z1 - z0
    part.visual(
        Cylinder(radius=radius, length=sqrt(dy * dy + dz * dz)),
        origin=Origin(
            xyz=(x, (y0 + y1) * 0.5, (z0 + z1) * 0.5),
            rpy=(-atan2(dy, dz), 0.0, 0.0),
        ),
        material=material,
        name=name,
    )


def _build_hinge_link(model: ArticulatedObject, name: str, *, side_sign: float, hinge_material, fastener_material):
    part = model.part(name)
    part.inertial = Inertial.from_geometry(
        Box((0.09, 0.54, 0.09)),
        mass=0.35,
        origin=Origin(xyz=(-side_sign * 0.018, 0.0, 0.0)),
    )
    part.visual(
        Box((0.016, 0.36, 0.024)),
        origin=Origin(xyz=(-side_sign * 0.026, 0.0, 0.0)),
        material=hinge_material,
        name="clamp_saddle",
    )
    part.visual(
        Box((0.020, 0.52, 0.070)),
        origin=Origin(xyz=(-side_sign * 0.022, 0.0, 0.0)),
        material=hinge_material,
        name="backplate",
    )
    part.visual(
        Box((0.012, 0.40, 0.018)),
        origin=Origin(xyz=(-side_sign * 0.018, 0.0, -0.014)),
        material=hinge_material,
        name="bridge_arm",
    )
    part.visual(
        Box((0.030, 0.012, 0.054)),
        origin=Origin(xyz=(-side_sign * 0.005, WING_HALF_DEPTH + 0.006, 0.0)),
        material=hinge_material,
        name="front_cheek",
    )
    part.visual(
        Box((0.030, 0.012, 0.054)),
        origin=Origin(xyz=(-side_sign * 0.005, -(WING_HALF_DEPTH + 0.006), 0.0)),
        material=hinge_material,
        name="rear_cheek",
    )
    part.visual(
        Box((0.028, 0.10, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material=hinge_material,
        name="lower_stop",
    )
    part.visual(
        Box((0.024, 0.10, 0.010)),
        origin=Origin(xyz=(-side_sign * 0.010, 0.0, 0.025)),
        material=hinge_material,
        name="upper_guide",
    )
    for visual_name, y_pos in (("front_bolt", 0.14), ("rear_bolt", -0.14)):
        part.visual(
            Cylinder(radius=0.006, length=0.020),
            origin=Origin(xyz=(-side_sign * 0.028, y_pos, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=fastener_material,
            name=visual_name,
        )
    return part


def _build_wing(model: ArticulatedObject, name: str, *, side_sign: float, frame_material, stop_material):
    part = model.part(name)
    part.inertial = Inertial.from_geometry(
        Box((WING_WIDTH, 0.52, 0.05)),
        mass=1.4,
        origin=Origin(xyz=(side_sign * WING_WIDTH * 0.5, 0.0, 0.0)),
    )
    _add_rod_y(
        part,
        "hinge_barrel",
        x=0.0,
        y0=-WING_HALF_DEPTH,
        y1=WING_HALF_DEPTH,
        z=0.0,
        radius=HINGE_BARREL_RADIUS,
        material=frame_material,
    )
    _add_rod_y(
        part,
        "outer_rail",
        x=side_sign * WING_WIDTH,
        y0=-WING_HALF_DEPTH,
        y1=WING_HALF_DEPTH,
        z=0.0,
        radius=TUBE_RADIUS,
        material=frame_material,
    )
    _add_rod_x(
        part,
        "front_frame_rail",
        x0=0.0,
        x1=side_sign * WING_WIDTH,
        y=0.23,
        z=0.0,
        radius=TUBE_RADIUS,
        material=frame_material,
    )
    _add_rod_x(
        part,
        "rear_frame_rail",
        x0=0.0,
        x1=side_sign * WING_WIDTH,
        y=-0.23,
        z=0.0,
        radius=TUBE_RADIUS,
        material=frame_material,
    )
    for index, x_pos in enumerate((0.10, 0.20, 0.30, 0.40), start=1):
        _add_rod_y(
            part,
            f"hanging_rail_{index}",
            x=side_sign * x_pos,
            y0=-0.23,
            y1=0.23,
            z=0.0,
            radius=RAIL_RADIUS,
            material=frame_material,
        )
    part.visual(
        Box((0.020, 0.10, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=stop_material,
        name="lower_stop_pad",
    )
    part.visual(
        Box((0.018, 0.08, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=stop_material,
        name="upper_stop_pad",
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="simple_drying_rack")

    frame_powder = model.material("frame_powder", rgba=(0.86, 0.87, 0.88, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.27, 0.29, 0.31, 1.0))
    foot_gray = model.material("foot_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    fastener = model.material("fastener", rgba=(0.65, 0.67, 0.70, 1.0))

    main_frame = model.part("main_frame")
    main_frame.inertial = Inertial.from_geometry(
        Box((0.98, 0.68, 0.92)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
    )

    _add_rod_y(
        main_frame,
        "left_side_rail",
        x=DECK_HALF_WIDTH,
        y0=-DECK_HALF_DEPTH,
        y1=DECK_HALF_DEPTH,
        z=DECK_HEIGHT,
        radius=TUBE_RADIUS,
        material=frame_powder,
    )
    _add_rod_y(
        main_frame,
        "right_side_rail",
        x=-DECK_HALF_WIDTH,
        y0=-DECK_HALF_DEPTH,
        y1=DECK_HALF_DEPTH,
        z=DECK_HEIGHT,
        radius=TUBE_RADIUS,
        material=frame_powder,
    )
    _add_rod_x(
        main_frame,
        "front_top_rail",
        x0=-DECK_HALF_WIDTH,
        x1=DECK_HALF_WIDTH,
        y=DECK_HALF_DEPTH,
        z=DECK_HEIGHT,
        radius=TUBE_RADIUS,
        material=frame_powder,
    )
    _add_rod_x(
        main_frame,
        "rear_top_rail",
        x0=-DECK_HALF_WIDTH,
        x1=DECK_HALF_WIDTH,
        y=-DECK_HALF_DEPTH,
        z=DECK_HEIGHT,
        radius=TUBE_RADIUS,
        material=frame_powder,
    )
    for index, x_pos in enumerate((-0.22, -0.11, 0.0, 0.11, 0.22), start=1):
        _add_rod_y(
            main_frame,
            f"center_hanging_rail_{index}",
            x=x_pos,
            y0=-DECK_HALF_DEPTH,
            y1=DECK_HALF_DEPTH,
            z=DECK_HEIGHT,
            radius=RAIL_RADIUS,
            material=frame_powder,
        )

    _add_leg_yz(
        main_frame,
        "front_left_leg",
        x=DECK_HALF_WIDTH,
        y0=0.31,
        z0=0.03,
        y1=DECK_HALF_DEPTH,
        z1=DECK_HEIGHT - 0.01,
        radius=TUBE_RADIUS,
        material=frame_powder,
    )
    _add_leg_yz(
        main_frame,
        "rear_left_leg",
        x=DECK_HALF_WIDTH,
        y0=-0.31,
        z0=0.03,
        y1=-DECK_HALF_DEPTH,
        z1=DECK_HEIGHT - 0.01,
        radius=TUBE_RADIUS,
        material=frame_powder,
    )
    _add_leg_yz(
        main_frame,
        "front_right_leg",
        x=-DECK_HALF_WIDTH,
        y0=0.31,
        z0=0.03,
        y1=DECK_HALF_DEPTH,
        z1=DECK_HEIGHT - 0.01,
        radius=TUBE_RADIUS,
        material=frame_powder,
    )
    _add_leg_yz(
        main_frame,
        "rear_right_leg",
        x=-DECK_HALF_WIDTH,
        y0=-0.31,
        z0=0.03,
        y1=-DECK_HALF_DEPTH,
        z1=DECK_HEIGHT - 0.01,
        radius=TUBE_RADIUS,
        material=frame_powder,
    )
    _add_rod_y(
        main_frame,
        "left_lower_tie",
        x=DECK_HALF_WIDTH,
        y0=-0.29,
        y1=0.29,
        z=0.36,
        radius=RAIL_RADIUS,
        material=frame_powder,
    )
    _add_rod_y(
        main_frame,
        "right_lower_tie",
        x=-DECK_HALF_WIDTH,
        y0=-0.29,
        y1=0.29,
        z=0.36,
        radius=RAIL_RADIUS,
        material=frame_powder,
    )
    _add_rod_x(
        main_frame,
        "front_stabilizer",
        x0=-DECK_HALF_WIDTH,
        x1=DECK_HALF_WIDTH,
        y=0.28,
        z=0.22,
        radius=RAIL_RADIUS,
        material=frame_powder,
    )
    _add_rod_x(
        main_frame,
        "rear_stabilizer",
        x0=-DECK_HALF_WIDTH,
        x1=DECK_HALF_WIDTH,
        y=-0.28,
        z=0.22,
        radius=RAIL_RADIUS,
        material=frame_powder,
    )
    for visual_name, x_pos, y_pos in (
        ("front_left_foot", DECK_HALF_WIDTH, 0.31),
        ("rear_left_foot", DECK_HALF_WIDTH, -0.31),
        ("front_right_foot", -DECK_HALF_WIDTH, 0.31),
        ("rear_right_foot", -DECK_HALF_WIDTH, -0.31),
    ):
        main_frame.visual(
            Box((0.05, 0.025, 0.030)),
            origin=Origin(xyz=(x_pos, y_pos, 0.015)),
            material=foot_gray,
            name=visual_name,
        )

    left_hinge_link = _build_hinge_link(
        model,
        "left_hinge_link",
        side_sign=1.0,
        hinge_material=hinge_dark,
        fastener_material=fastener,
    )
    right_hinge_link = _build_hinge_link(
        model,
        "right_hinge_link",
        side_sign=-1.0,
        hinge_material=hinge_dark,
        fastener_material=fastener,
    )
    left_wing = _build_wing(
        model,
        "left_wing_frame",
        side_sign=1.0,
        frame_material=frame_powder,
        stop_material=hinge_dark,
    )
    right_wing = _build_wing(
        model,
        "right_wing_frame",
        side_sign=-1.0,
        frame_material=frame_powder,
        stop_material=hinge_dark,
    )

    model.articulation(
        "main_to_left_hinge",
        ArticulationType.FIXED,
        parent=main_frame,
        child=left_hinge_link,
        origin=Origin(xyz=(DECK_HALF_WIDTH + HINGE_OFFSET, 0.0, DECK_HEIGHT)),
    )
    model.articulation(
        "main_to_right_hinge",
        ArticulationType.FIXED,
        parent=main_frame,
        child=right_hinge_link,
        origin=Origin(xyz=(-(DECK_HALF_WIDTH + HINGE_OFFSET), 0.0, DECK_HEIGHT)),
    )
    model.articulation(
        "left_wing_hinge",
        ArticulationType.REVOLUTE,
        parent=left_hinge_link,
        child=left_wing,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.30),
    )
    model.articulation(
        "right_wing_hinge",
        ArticulationType.REVOLUTE,
        parent=right_hinge_link,
        child=right_wing,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    main_frame = object_model.get_part("main_frame")
    left_hinge_link = object_model.get_part("left_hinge_link")
    right_hinge_link = object_model.get_part("right_hinge_link")
    left_wing = object_model.get_part("left_wing_frame")
    right_wing = object_model.get_part("right_wing_frame")
    left_hinge = object_model.get_articulation("left_wing_hinge")
    right_hinge = object_model.get_articulation("right_wing_hinge")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        left_hinge_link,
        main_frame,
        elem_a="clamp_saddle",
        elem_b="left_side_rail",
        name="left_hinge_clamp_contacts_main_frame",
    )
    ctx.expect_contact(
        right_hinge_link,
        main_frame,
        elem_a="clamp_saddle",
        elem_b="right_side_rail",
        name="right_hinge_clamp_contacts_main_frame",
    )
    ctx.expect_contact(
        left_wing,
        left_hinge_link,
        elem_a="hinge_barrel",
        elem_b="front_cheek",
        name="left_wing_retained_by_hinge_cheek",
    )
    ctx.expect_contact(
        right_wing,
        right_hinge_link,
        elem_a="hinge_barrel",
        elem_b="front_cheek",
        name="right_wing_retained_by_hinge_cheek",
    )

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        ctx.expect_contact(
            left_wing,
            left_hinge_link,
            elem_a="lower_stop_pad",
            elem_b="lower_stop",
            name="left_wing_open_stop_contacts",
        )
        ctx.expect_contact(
            right_wing,
            right_hinge_link,
            elem_a="lower_stop_pad",
            elem_b="lower_stop",
            name="right_wing_open_stop_contacts",
        )

    with ctx.pose({left_hinge: 1.30, right_hinge: 1.30}):
        left_outer_aabb = ctx.part_element_world_aabb(left_wing, elem="outer_rail")
        left_hinge_aabb = ctx.part_element_world_aabb(left_wing, elem="hinge_barrel")
        right_outer_aabb = ctx.part_element_world_aabb(right_wing, elem="outer_rail")
        right_hinge_aabb = ctx.part_element_world_aabb(right_wing, elem="hinge_barrel")

        if None in (left_outer_aabb, left_hinge_aabb, right_outer_aabb, right_hinge_aabb):
            ctx.fail("folded_wing_raise_check_available", "could not resolve wing element AABBs in folded pose")
        else:
            left_outer_center_z = (left_outer_aabb[0][2] + left_outer_aabb[1][2]) * 0.5
            left_hinge_center_z = (left_hinge_aabb[0][2] + left_hinge_aabb[1][2]) * 0.5
            right_outer_center_z = (right_outer_aabb[0][2] + right_outer_aabb[1][2]) * 0.5
            right_hinge_center_z = (right_hinge_aabb[0][2] + right_hinge_aabb[1][2]) * 0.5
            ctx.check(
                "folded_wings_raise_outer_rails",
                (left_outer_center_z - left_hinge_center_z) > 0.35
                and (right_outer_center_z - right_hinge_center_z) > 0.35,
                details=(
                    f"left dz={left_outer_center_z - left_hinge_center_z:.3f}, "
                    f"right dz={right_outer_center_z - right_hinge_center_z:.3f}"
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
