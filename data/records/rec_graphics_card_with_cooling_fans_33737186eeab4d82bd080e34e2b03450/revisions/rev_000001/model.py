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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


CARD_LENGTH = 0.285
CARD_HEIGHT = 0.112
CARD_THICKNESS = 0.036
LEFT_FAN_X = -0.064
RIGHT_FAN_X = 0.058
FAN_Y = 0.0
FAN_JOINT_Z = 0.0085


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _regular_profile(
    *,
    center: tuple[float, float],
    radius_x: float,
    radius_y: float,
    sides: int,
    angle_offset: float = 0.0,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius_x * math.cos(angle_offset + (math.tau * i / sides)),
            cy + radius_y * math.sin(angle_offset + (math.tau * i / sides)),
        )
        for i in range(sides)
    ]


def _build_front_shroud_mesh():
    outer_profile = [
        (-0.138, -0.054),
        (0.106, -0.054),
        (0.132, -0.045),
        (0.140, -0.024),
        (0.140, 0.040),
        (0.127, 0.054),
        (-0.128, 0.054),
        (-0.140, 0.043),
        (-0.140, -0.040),
    ]
    left_hole = _regular_profile(
        center=(LEFT_FAN_X, FAN_Y),
        radius_x=0.044,
        radius_y=0.044,
        sides=12,
        angle_offset=math.pi / 12.0,
    )
    right_hole = _regular_profile(
        center=(RIGHT_FAN_X, FAN_Y),
        radius_x=0.044,
        radius_y=0.044,
        sides=12,
        angle_offset=math.pi / 12.0,
    )
    return ExtrudeWithHolesGeometry(
        outer_profile,
        [left_hole, right_hole],
        height=0.008,
        center=True,
    )


def _build_rotor_ring_mesh():
    return ExtrudeWithHolesGeometry(
        _regular_profile(center=(0.0, 0.0), radius_x=0.040, radius_y=0.040, sides=28),
        [_regular_profile(center=(0.0, 0.0), radius_x=0.034, radius_y=0.034, sides=28)],
        height=0.006,
        center=True,
    )


def _build_fan_blade_mesh():
    return ExtrudeGeometry(
        [
            (-0.016, -0.0035),
            (-0.008, 0.0005),
            (0.002, 0.0038),
            (0.016, 0.0020),
            (0.010, -0.0040),
            (-0.002, -0.0050),
        ],
        height=0.003,
        center=True,
    )


def _add_rotor(
    part,
    *,
    ring_mesh,
    blade_mesh,
    frame_material,
    blade_material,
    hub_material,
    prefix: str,
) -> None:
    part.visual(ring_mesh, material=frame_material, name="rotor_ring")
    part.visual(
        Cylinder(radius=0.0125, length=0.007),
        material=hub_material,
        name="rotor_hub",
    )
    part.visual(
        Cylinder(radius=0.0065, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.0005)),
        material=frame_material,
        name="rotor_cap",
    )

    blade_count = 7
    for index in range(blade_count):
        angle = math.tau * index / blade_count
        radius = 0.0215
        part.visual(
            blade_mesh,
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.0),
                rpy=(0.0, 0.0, angle + 0.34),
            ),
            material=blade_material,
            name=f"{prefix}_blade_{index:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_fan_graphics_card")

    shroud_black = model.material("shroud_black", rgba=(0.09, 0.10, 0.11, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.16, 0.17, 0.19, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.62, 0.65, 0.69, 1.0))
    fan_black = model.material("fan_black", rgba=(0.05, 0.05, 0.06, 1.0))
    blade_gray = model.material("blade_gray", rgba=(0.27, 0.29, 0.31, 1.0))
    connector_gray = model.material("connector_gray", rgba=(0.20, 0.22, 0.24, 1.0))
    bracket_steel = model.material("bracket_steel", rgba=(0.72, 0.74, 0.77, 1.0))

    front_shroud_mesh = _save_mesh("gpu_front_shroud", _build_front_shroud_mesh())
    rotor_ring_mesh = _save_mesh("gpu_rotor_ring", _build_rotor_ring_mesh())
    fan_blade_mesh = _save_mesh("gpu_fan_blade", _build_fan_blade_mesh())

    body = model.part("card_body")
    body.visual(
        Box((0.270, 0.092, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=satin_metal,
        name="heatsink_block",
    )
    body.visual(
        Box((0.276, 0.106, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=dark_graphite,
        name="backplate",
    )
    body.visual(
        front_shroud_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=shroud_black,
        name="front_shroud",
    )
    body.visual(
        Box((0.248, 0.012, 0.010)),
        origin=Origin(xyz=(-0.005, 0.049, 0.007)),
        material=dark_graphite,
        name="top_rail",
    )
    body.visual(
        Box((0.280, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.050, 0.007)),
        material=dark_graphite,
        name="bottom_rail",
    )
    body.visual(
        Box((0.028, 0.040, 0.010)),
        origin=Origin(xyz=(0.124, 0.033, 0.007)),
        material=dark_graphite,
        name="right_end_cap",
    )
    body.visual(
        Box((0.022, 0.052, 0.010)),
        origin=Origin(xyz=(-0.128, 0.002, 0.007)),
        material=dark_graphite,
        name="left_shoulder",
    )
    body.visual(
        Box((0.012, 0.105, 0.036)),
        origin=Origin(xyz=(-0.141, 0.0, -0.001)),
        material=bracket_steel,
        name="io_bracket",
    )
    body.visual(
        Box((0.044, 0.014, 0.010)),
        origin=Origin(xyz=(0.094, 0.046, 0.001)),
        material=dark_graphite,
        name="connector_bed",
    )
    body.visual(
        Box((0.030, 0.010, 0.018)),
        origin=Origin(xyz=(0.094, 0.051, 0.0)),
        material=connector_gray,
        name="power_socket",
    )
    body.visual(
        Box((0.050, 0.002, 0.005)),
        origin=Origin(xyz=(0.094, 0.0545, 0.017)),
        material=connector_gray,
        name="flap_hinge_saddle",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.005),
        origin=Origin(xyz=(LEFT_FAN_X, FAN_Y, 0.0025)),
        material=connector_gray,
        name="left_bearing_post",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.005),
        origin=Origin(xyz=(RIGHT_FAN_X, FAN_Y, 0.0025)),
        material=connector_gray,
        name="right_bearing_post",
    )
    body.inertial = Inertial.from_geometry(
        Box((CARD_LENGTH, CARD_HEIGHT, CARD_THICKNESS)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    left_fan = model.part("left_fan")
    _add_rotor(
        left_fan,
        ring_mesh=rotor_ring_mesh,
        blade_mesh=fan_blade_mesh,
        frame_material=fan_black,
        blade_material=blade_gray,
        hub_material=connector_gray,
        prefix="left",
    )
    left_fan.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.008),
        mass=0.08,
    )

    right_fan = model.part("right_fan")
    _add_rotor(
        right_fan,
        ring_mesh=rotor_ring_mesh,
        blade_mesh=fan_blade_mesh,
        frame_material=fan_black,
        blade_material=blade_gray,
        hub_material=connector_gray,
        prefix="right",
    )
    right_fan.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.008),
        mass=0.08,
    )

    flap = model.part("power_flap")
    flap.visual(
        Cylinder(radius=0.0025, length=0.046),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=connector_gray,
        name="flap_barrel",
    )
    flap.visual(
        Box((0.050, 0.003, 0.034)),
        origin=Origin(xyz=(0.0, 0.0015, -0.017)),
        material=shroud_black,
        name="flap_panel",
    )
    flap.visual(
        Box((0.018, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, 0.0015, -0.032)),
        material=dark_graphite,
        name="flap_tab",
    )
    flap.inertial = Inertial.from_geometry(
        Box((0.052, 0.008, 0.036)),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0015, -0.016)),
    )

    model.articulation(
        "left_fan_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_fan,
        origin=Origin(xyz=(LEFT_FAN_X, FAN_Y, FAN_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=180.0),
    )
    model.articulation(
        "right_fan_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_fan,
        origin=Origin(xyz=(RIGHT_FAN_X, FAN_Y, FAN_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=180.0),
    )
    model.articulation(
        "power_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.094, 0.058, 0.017)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(88.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("card_body")
    left_fan = object_model.get_part("left_fan")
    right_fan = object_model.get_part("right_fan")
    flap = object_model.get_part("power_flap")

    left_spin = object_model.get_articulation("left_fan_spin")
    right_spin = object_model.get_articulation("right_fan_spin")
    flap_hinge = object_model.get_articulation("power_flap_hinge")

    ctx.check(
        "fan spin joints are continuous and normal to the card face",
        left_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_spin.articulation_type == ArticulationType.CONTINUOUS
        and left_spin.axis == (0.0, 0.0, 1.0)
        and right_spin.axis == (0.0, 0.0, 1.0),
        details=(
            f"left type/axis={left_spin.articulation_type}/{left_spin.axis}, "
            f"right type/axis={right_spin.articulation_type}/{right_spin.axis}"
        ),
    )
    ctx.check(
        "power flap hinge runs along the top edge",
        flap_hinge.axis == (1.0, 0.0, 0.0),
        details=f"axis={flap_hinge.axis}",
    )

    ctx.expect_overlap(
        left_fan,
        body,
        axes="xy",
        elem_a="rotor_ring",
        elem_b="front_shroud",
        min_overlap=0.070,
        name="left fan sits inside the shroud footprint",
    )
    ctx.expect_overlap(
        right_fan,
        body,
        axes="xy",
        elem_a="rotor_ring",
        elem_b="front_shroud",
        min_overlap=0.070,
        name="right fan sits inside the shroud footprint",
    )
    ctx.expect_overlap(
        flap,
        body,
        axes="xz",
        elem_a="flap_panel",
        elem_b="power_socket",
        min_overlap=0.018,
        name="closed flap covers the power socket",
    )

    with ctx.pose({flap_hinge: 0.0}):
        ctx.expect_gap(
            flap,
            body,
            axis="y",
            positive_elem="flap_panel",
            negative_elem="power_socket",
            min_gap=0.0002,
            max_gap=0.004,
            name="closed flap sits just above the socket",
        )
        closed_panel = ctx.part_element_world_aabb(flap, elem="flap_panel")

    with ctx.pose({flap_hinge: math.radians(80.0)}):
        ctx.expect_gap(
            flap,
            body,
            axis="y",
            positive_elem="flap_tab",
            negative_elem="power_socket",
            min_gap=0.010,
            name="opened flap tab lifts clear of the socket",
        )
        open_panel = ctx.part_element_world_aabb(flap, elem="flap_panel")

    ctx.check(
        "opened flap raises above its closed position",
        closed_panel is not None
        and open_panel is not None
        and open_panel[1][1] > closed_panel[1][1] + 0.012,
        details=f"closed={closed_panel}, open={open_panel}",
    )

    left_pos = ctx.part_world_position(left_fan)
    right_pos = ctx.part_world_position(right_fan)
    ctx.check(
        "dual fans are spaced side by side across the card",
        left_pos is not None
        and right_pos is not None
        and left_pos[0] < right_pos[0] - 0.08
        and abs(left_pos[1] - right_pos[1]) < 0.005,
        details=f"left={left_pos}, right={right_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
