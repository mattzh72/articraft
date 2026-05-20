from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_dual_fan_card")

    shroud_black = model.material("shroud_black", rgba=(0.10, 0.11, 0.12, 1.0))
    rotor_black = model.material("rotor_black", rgba=(0.06, 0.06, 0.07, 1.0))
    heatsink_aluminum = model.material("heatsink_aluminum", rgba=(0.68, 0.70, 0.73, 1.0))
    bracket_silver = model.material("bracket_silver", rgba=(0.78, 0.79, 0.81, 1.0))
    pcb_green = model.material("pcb_green", rgba=(0.10, 0.34, 0.18, 1.0))
    dark_io = model.material("dark_io", rgba=(0.16, 0.17, 0.19, 1.0))
    flap_gray = model.material("flap_gray", rgba=(0.26, 0.27, 0.29, 1.0))

    def circle_profile(
        radius: float,
        *,
        center: tuple[float, float] = (0.0, 0.0),
        segments: int = 28,
        clockwise: bool = False,
    ) -> list[tuple[float, float]]:
        cx, cy = center
        points: list[tuple[float, float]] = []
        for index in range(segments):
            angle = math.tau * index / segments
            if clockwise:
                angle = -angle
            points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
        return points

    def build_shroud_face() -> MeshGeometry:
        fan_opening_radius = 0.032
        outer = rounded_rect_profile(0.166, 0.102, 0.006, corner_segments=6)
        left_hole = circle_profile(fan_opening_radius, center=(-0.040, 0.0), clockwise=True)
        right_hole = circle_profile(fan_opening_radius, center=(0.040, 0.0), clockwise=True)
        return ExtrudeWithHolesGeometry(
            outer,
            [left_hole, right_hole],
            0.004,
            cap=True,
            center=True,
            closed=True,
        )

    def build_fan_frame(center_x: float) -> MeshGeometry:
        return ExtrudeWithHolesGeometry(
            circle_profile(0.0355, center=(center_x, 0.0), segments=40),
            [circle_profile(0.0305, center=(center_x, 0.0), segments=40, clockwise=True)],
            0.002,
            cap=True,
            center=True,
            closed=True,
        )

    def build_fan_rotor(name: str):
        rotor = MeshGeometry()
        rotor.merge(CylinderGeometry(radius=0.012, height=0.008, radial_segments=24))
        rotor.merge(
            CylinderGeometry(radius=0.016, height=0.002, radial_segments=24).translate(0.0, 0.0, 0.004)
        )

        blade_count = 9
        for index in range(blade_count):
            blade = BoxGeometry((0.028, 0.008, 0.0024))
            blade.translate(0.017, 0.0, 0.0)
            blade.rotate_z((math.tau * index) / blade_count + math.radians(16.0))
            rotor.merge(blade)

        return mesh_from_geometry(rotor, name), Box((0.050, 0.050, 0.010))

    card_body = model.part("card_body")
    card_body.visual(
        Box((0.182, 0.100, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=pcb_green,
        name="pcb",
    )
    card_body.visual(
        Box((0.154, 0.090, 0.022)),
        origin=Origin(xyz=(0.010, 0.0, 0.013)),
        material=heatsink_aluminum,
        name="heatsink_core",
    )

    for index in range(10):
        y_pos = -0.036 + index * 0.008
        card_body.visual(
            Box((0.148, 0.003, 0.015)),
            origin=Origin(xyz=(0.012, y_pos, 0.0315)),
            material=heatsink_aluminum,
            name=f"fin_{index + 1}",
        )

    card_body.visual(
        mesh_from_geometry(build_shroud_face(), "shroud_face"),
        origin=Origin(xyz=(0.003, 0.0, 0.042)),
        material=shroud_black,
        name="shroud_face",
    )
    card_body.visual(
        mesh_from_geometry(build_fan_frame(-0.040), "left_fan_frame"),
        origin=Origin(xyz=(0.003, 0.0, 0.039)),
        material=dark_io,
        name="left_fan_frame",
    )
    card_body.visual(
        mesh_from_geometry(build_fan_frame(0.040), "right_fan_frame"),
        origin=Origin(xyz=(0.003, 0.0, 0.039)),
        material=dark_io,
        name="right_fan_frame",
    )
    card_body.visual(
        Box((0.166, 0.006, 0.016)),
        origin=Origin(xyz=(0.003, 0.048, 0.032)),
        material=shroud_black,
        name="upper_side_rail",
    )
    card_body.visual(
        Box((0.166, 0.006, 0.016)),
        origin=Origin(xyz=(0.003, -0.048, 0.032)),
        material=shroud_black,
        name="lower_side_rail",
    )
    card_body.visual(
        Box((0.006, 0.090, 0.016)),
        origin=Origin(xyz=(0.003, 0.0, 0.032)),
        material=shroud_black,
        name="center_bridge",
    )
    card_body.visual(
        Box((0.006, 0.090, 0.016)),
        origin=Origin(xyz=(-0.077, 0.0, 0.032)),
        material=shroud_black,
        name="rear_bridge",
    )
    card_body.visual(
        Box((0.006, 0.090, 0.016)),
        origin=Origin(xyz=(0.083, 0.0, 0.032)),
        material=shroud_black,
        name="nose_bridge",
    )
    card_body.visual(
        Box((0.0025, 0.112, 0.046)),
        origin=Origin(xyz=(-0.09225, 0.0, 0.023)),
        material=bracket_silver,
        name="rear_bracket",
    )
    card_body.visual(
        Box((0.012, 0.046, 0.018)),
        origin=Origin(xyz=(-0.086, -0.022, 0.021)),
        material=dark_io,
        name="io_block",
    )
    card_body.visual(
        Box((0.024, 0.022, 0.006)),
        origin=Origin(xyz=(-0.070, 0.031, 0.047)),
        material=shroud_black,
        name="latch_pedestal",
    )
    card_body.visual(
        Cylinder(radius=0.0025, length=0.005),
        origin=Origin(xyz=(-0.081, 0.0245, 0.050), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bracket_silver,
        name="hinge_knuckle_a",
    )
    card_body.visual(
        Cylinder(radius=0.0025, length=0.005),
        origin=Origin(xyz=(-0.081, 0.0375, 0.050), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bracket_silver,
        name="hinge_knuckle_b",
    )
    card_body.inertial = Inertial.from_geometry(
        Box((0.185, 0.112, 0.052)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    left_fan = model.part("left_fan")
    left_rotor_mesh, left_rotor_proxy = build_fan_rotor("left_fan_rotor")
    left_fan.visual(left_rotor_mesh, material=rotor_black, name="rotor_assembly")
    left_fan.inertial = Inertial.from_geometry(left_rotor_proxy, mass=0.06)

    right_fan = model.part("right_fan")
    right_rotor_mesh, right_rotor_proxy = build_fan_rotor("right_fan_rotor")
    right_fan.visual(right_rotor_mesh, material=rotor_black, name="rotor_assembly")
    right_fan.inertial = Inertial.from_geometry(right_rotor_proxy, mass=0.06)

    cover_flap = model.part("cover_flap")
    cover_flap.visual(
        Cylinder(radius=0.0023, length=0.008),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bracket_silver,
        name="hinge_barrel",
    )
    cover_flap.visual(
        Box((0.028, 0.020, 0.0025)),
        origin=Origin(xyz=(0.014, 0.0, 0.00125)),
        material=flap_gray,
        name="cover_panel",
    )
    cover_flap.visual(
        Box((0.004, 0.016, 0.003)),
        origin=Origin(xyz=(0.027, 0.0, 0.003)),
        material=flap_gray,
        name="pull_lip",
    )
    cover_flap.inertial = Inertial.from_geometry(
        Box((0.030, 0.020, 0.006)),
        mass=0.02,
        origin=Origin(xyz=(0.015, 0.0, 0.002)),
    )

    model.articulation(
        "body_to_left_fan",
        ArticulationType.CONTINUOUS,
        parent=card_body,
        child=left_fan,
        origin=Origin(xyz=(-0.037, 0.0, 0.032)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=45.0),
    )
    model.articulation(
        "body_to_right_fan",
        ArticulationType.CONTINUOUS,
        parent=card_body,
        child=right_fan,
        origin=Origin(xyz=(0.043, 0.0, 0.032)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=45.0),
    )
    model.articulation(
        "body_to_cover_flap",
        ArticulationType.REVOLUTE,
        parent=card_body,
        child=cover_flap,
        origin=Origin(xyz=(-0.081, 0.031, 0.050)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=1.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    card_body = object_model.get_part("card_body")
    left_fan = object_model.get_part("left_fan")
    right_fan = object_model.get_part("right_fan")
    cover_flap = object_model.get_part("cover_flap")

    left_spin = object_model.get_articulation("body_to_left_fan")
    right_spin = object_model.get_articulation("body_to_right_fan")
    flap_joint = object_model.get_articulation("body_to_cover_flap")

    ctx.check(
        "left fan uses a vertical continuous spin axis",
        left_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={left_spin.articulation_type}, axis={left_spin.axis}",
    )
    ctx.check(
        "right fan uses a vertical continuous spin axis",
        right_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(right_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={right_spin.articulation_type}, axis={right_spin.axis}",
    )
    ctx.check(
        "cover flap hinges upward from the rear bracket side",
        flap_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(flap_joint.axis) == (0.0, -1.0, 0.0)
        and flap_joint.motion_limits is not None
        and flap_joint.motion_limits.lower == 0.0
        and flap_joint.motion_limits.upper is not None
        and flap_joint.motion_limits.upper >= 1.0,
        details=(
            f"type={flap_joint.articulation_type}, axis={flap_joint.axis}, "
            f"limits={flap_joint.motion_limits}"
        ),
    )

    ctx.expect_origin_gap(
        right_fan,
        left_fan,
        axis="x",
        min_gap=0.075,
        max_gap=0.085,
        name="fan centers are spaced for a compact dual-fan layout",
    )

    ctx.expect_gap(
        card_body,
        left_fan,
        axis="z",
        positive_elem="shroud_face",
        negative_elem="rotor_assembly",
        min_gap=0.002,
        max_gap=0.010,
        name="left rotor sits just below the shroud face",
    )
    ctx.expect_gap(
        card_body,
        right_fan,
        axis="z",
        positive_elem="shroud_face",
        negative_elem="rotor_assembly",
        min_gap=0.002,
        max_gap=0.010,
        name="right rotor sits just below the shroud face",
    )
    ctx.expect_gap(
        cover_flap,
        card_body,
        axis="z",
        positive_elem="cover_panel",
        negative_elem="latch_pedestal",
        min_gap=0.0,
        max_gap=0.0015,
        name="closed cover flap rests over the latch pedestal",
    )

    closed_cover_aabb = ctx.part_element_world_aabb(cover_flap, elem="cover_panel")
    flap_upper = flap_joint.motion_limits.upper if flap_joint.motion_limits is not None else None
    opened_cover_aabb = None
    if flap_upper is not None:
        with ctx.pose({flap_joint: flap_upper}):
            opened_cover_aabb = ctx.part_element_world_aabb(cover_flap, elem="cover_panel")

    ctx.check(
        "cover flap opens upward away from the latch area",
        closed_cover_aabb is not None
        and opened_cover_aabb is not None
        and opened_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.015,
        details=f"closed={closed_cover_aabb}, opened={opened_cover_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
