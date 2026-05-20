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


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_segment(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _lerp(
    a: tuple[float, float, float], b: tuple[float, float, float], t: float
) -> tuple[float, float, float]:
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def _add_box_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    width: float,
    depth: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((width, depth, _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_segment(a, b)),
        material=material,
        name=name,
    )


def _add_round_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_segment(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hybrid_step_extension_ladder")

    aluminum = model.material("aluminum", rgba=(0.80, 0.82, 0.84, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.68, 0.71, 0.74, 1.0))
    fiberglass_orange = model.material("fiberglass_orange", rgba=(0.95, 0.44, 0.10, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.10, 0.10, 0.11, 1.0))
    warning_red = model.material("warning_red", rgba=(0.77, 0.18, 0.12, 1.0))

    front_frame = model.part("front_frame")

    front_left_bottom = (-0.230, 0.000, 0.040)
    front_left_top = (-0.180, 0.000, 1.660)
    front_right_bottom = (0.230, 0.000, 0.040)
    front_right_top = (0.180, 0.000, 1.660)

    _add_box_member(
        front_frame,
        front_left_bottom,
        front_left_top,
        width=0.062,
        depth=0.032,
        material=aluminum,
        name="left_front_rail",
    )
    _add_box_member(
        front_frame,
        front_right_bottom,
        front_right_top,
        width=0.062,
        depth=0.032,
        material=aluminum,
        name="right_front_rail",
    )

    front_frame.visual(
        Box((0.400, 0.110, 0.060)),
        origin=Origin(xyz=(0.000, -0.040, 1.690)),
        material=brushed_steel,
        name="front_crown",
    )
    front_frame.visual(
        Box((0.340, 0.075, 0.025)),
        origin=Origin(xyz=(0.000, -0.060, 1.7325)),
        material=fiberglass_orange,
        name="tool_tray",
    )

    front_step_fractions = (0.18, 0.37, 0.56, 0.75)
    for step_index, t in enumerate(front_step_fractions, start=1):
        left_point = _lerp(front_left_bottom, front_left_top, t)
        right_point = _lerp(front_right_bottom, front_right_top, t)
        front_frame.visual(
            Box((right_point[0] - left_point[0] + 0.050, 0.060, 0.030)),
            origin=Origin(
                xyz=((left_point[0] + right_point[0]) * 0.5, -0.010, left_point[2])
            ),
            material=aluminum,
            name=f"front_step_{step_index}",
        )

    front_frame.visual(
        Box((0.085, 0.055, 0.040)),
        origin=Origin(xyz=(-0.230, 0.000, 0.020)),
        material=rubber_black,
        name="left_front_foot",
    )
    front_frame.visual(
        Box((0.085, 0.055, 0.040)),
        origin=Origin(xyz=(0.230, 0.000, 0.020)),
        material=rubber_black,
        name="right_front_foot",
    )
    front_frame.visual(
        Box((0.120, 0.026, 0.055)),
        origin=Origin(xyz=(-0.180, 0.013, 1.600)),
        material=warning_red,
        name="left_guide_cap",
    )
    front_frame.visual(
        Box((0.120, 0.026, 0.055)),
        origin=Origin(xyz=(0.180, 0.013, 1.600)),
        material=warning_red,
        name="right_guide_cap",
    )
    front_frame.inertial = Inertial.from_geometry(
        Box((0.560, 0.160, 1.760)),
        mass=12.0,
        origin=Origin(xyz=(0.000, -0.010, 0.880)),
    )

    rear_frame = model.part("rear_frame")

    rear_left_top = (-0.190, -0.110, -0.030)
    rear_left_bottom = (-0.240, -0.830, -1.650)
    rear_right_top = (0.190, -0.110, -0.030)
    rear_right_bottom = (0.240, -0.830, -1.650)

    _add_box_member(
        rear_frame,
        rear_left_top,
        rear_left_bottom,
        width=0.055,
        depth=0.028,
        material=aluminum,
        name="left_rear_rail",
    )
    _add_box_member(
        rear_frame,
        rear_right_top,
        rear_right_bottom,
        width=0.055,
        depth=0.028,
        material=aluminum,
        name="right_rear_rail",
    )
    rear_frame.visual(
        Box((0.360, 0.100, 0.055)),
        origin=Origin(xyz=(0.000, -0.145, 0.000)),
        material=brushed_steel,
        name="rear_crown",
    )

    rear_rung_fractions = (0.22, 0.43, 0.63, 0.82)
    for rung_index, t in enumerate(rear_rung_fractions, start=1):
        left_point = _lerp(rear_left_top, rear_left_bottom, t)
        right_point = _lerp(rear_right_top, rear_right_bottom, t)
        rear_frame.visual(
            Box((right_point[0] - left_point[0] + 0.038, 0.032, 0.018)),
            origin=Origin(
                xyz=((left_point[0] + right_point[0]) * 0.5, left_point[1], left_point[2])
            ),
            material=aluminum,
            name=f"rear_brace_{rung_index}",
        )

    rear_frame.visual(
        Box((0.080, 0.050, 0.040)),
        origin=Origin(xyz=(-0.240, -0.830, -1.670)),
        material=rubber_black,
        name="left_rear_foot",
    )
    rear_frame.visual(
        Box((0.080, 0.050, 0.040)),
        origin=Origin(xyz=(0.240, -0.830, -1.670)),
        material=rubber_black,
        name="right_rear_foot",
    )
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.560, 0.940, 1.760)),
        mass=8.0,
        origin=Origin(xyz=(0.000, -0.470, -0.840)),
    )

    fly_section = model.part("fly_section")

    fly_left_bottom = (-0.165, 0.044, -0.520)
    fly_left_top = (-0.135, 0.044, 1.100)
    fly_right_bottom = (0.165, 0.044, -0.520)
    fly_right_top = (0.135, 0.044, 1.100)

    _add_box_member(
        fly_section,
        fly_left_bottom,
        fly_left_top,
        width=0.050,
        depth=0.022,
        material=aluminum,
        name="left_fly_rail",
    )
    _add_box_member(
        fly_section,
        fly_right_bottom,
        fly_right_top,
        width=0.050,
        depth=0.022,
        material=aluminum,
        name="right_fly_rail",
    )

    fly_rung_fractions = (0.14, 0.32, 0.50, 0.68, 0.86)
    for rung_index, t in enumerate(fly_rung_fractions, start=1):
        left_point = _lerp(fly_left_bottom, fly_left_top, t)
        right_point = _lerp(fly_right_bottom, fly_right_top, t)
        _add_round_member(
            fly_section,
            (left_point[0] - 0.010, left_point[1], left_point[2]),
            (right_point[0] + 0.010, right_point[1], right_point[2]),
            radius=0.016,
            material=brushed_steel,
            name=f"fly_rung_{rung_index}",
        )

    fly_section.visual(
        Box((0.320, 0.045, 0.035)),
        origin=Origin(xyz=(0.000, 0.044, 1.1175)),
        material=brushed_steel,
        name="fly_top_cap",
    )
    fly_section.visual(
        Box((0.070, 0.028, 0.026)),
        origin=Origin(xyz=(-0.135, 0.044, 1.148)),
        material=rubber_black,
        name="left_fly_top_endcap",
    )
    fly_section.visual(
        Box((0.070, 0.028, 0.026)),
        origin=Origin(xyz=(0.135, 0.044, 1.148)),
        material=rubber_black,
        name="right_fly_top_endcap",
    )

    lower_guide_t = (-0.320 - fly_left_bottom[2]) / (fly_left_top[2] - fly_left_bottom[2])
    upper_guide_t = (0.180 - fly_left_bottom[2]) / (fly_left_top[2] - fly_left_bottom[2])
    guide_levels = (
        ("lower", lower_guide_t),
        ("upper", upper_guide_t),
    )
    for side_name, bottom_pt, top_pt in (
        ("left", fly_left_bottom, fly_left_top),
        ("right", fly_right_bottom, fly_right_top),
    ):
        for level_name, t in guide_levels:
            guide_point = _lerp(bottom_pt, top_pt, t)
            fly_section.visual(
                Box((0.078, 0.018, 0.070)),
                origin=Origin(xyz=(guide_point[0], 0.025, guide_point[2])),
                material=rubber_black,
                name=f"{side_name}_{level_name}_guide",
            )

    fly_section.inertial = Inertial.from_geometry(
        Box((0.440, 0.120, 1.700)),
        mass=5.0,
        origin=Origin(xyz=(0.000, 0.044, 0.320)),
    )

    model.articulation(
        "crown_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(0.000, 0.000, 1.690)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=-0.18,
            upper=0.45,
        ),
    )
    model.articulation(
        "fly_extension",
        ArticulationType.PRISMATIC,
        parent=front_frame,
        child=fly_section,
        origin=Origin(xyz=(0.000, 0.000, 1.580)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.45,
            lower=0.0,
            upper=0.90,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    fly_section = object_model.get_part("fly_section")

    crown_hinge = object_model.get_articulation("crown_hinge")
    fly_extension = object_model.get_articulation("fly_extension")

    front_crown = front_frame.get_visual("front_crown")
    rear_crown = rear_frame.get_visual("rear_crown")
    left_front_rail = front_frame.get_visual("left_front_rail")
    right_front_rail = front_frame.get_visual("right_front_rail")
    left_fly_rail = fly_section.get_visual("left_fly_rail")
    left_lower_guide = fly_section.get_visual("left_lower_guide")
    right_lower_guide = fly_section.get_visual("right_lower_guide")

    ctx.expect_contact(
        front_frame,
        rear_frame,
        elem_a=front_crown,
        elem_b=rear_crown,
        name="rear_frame_contacts_front_crown",
    )
    ctx.expect_contact(
        fly_section,
        front_frame,
        elem_a=left_lower_guide,
        elem_b=left_front_rail,
        name="left_fly_guide_contacts_front_rail",
    )
    ctx.expect_contact(
        fly_section,
        front_frame,
        elem_a=right_lower_guide,
        elem_b=right_front_rail,
        name="right_fly_guide_contacts_front_rail",
    )
    ctx.expect_gap(
        fly_section,
        front_frame,
        axis="y",
        positive_elem=left_fly_rail,
        negative_elem=left_front_rail,
        min_gap=0.010,
        max_gap=0.022,
        name="fly_rails_stand_off_from_front_rails",
    )
    ctx.expect_overlap(
        fly_section,
        front_frame,
        axes="x",
        min_overlap=0.260,
        name="fly_section_stays_nested_over_front_width",
    )

    rear_rest_aabb = ctx.part_world_aabb(rear_frame)
    assert rear_rest_aabb is not None
    with ctx.pose({crown_hinge: 0.35}):
        rear_closed_aabb = ctx.part_world_aabb(rear_frame)
        assert rear_closed_aabb is not None
        ctx.check(
            "crown_hinge_swings_rear_section_forward",
            rear_closed_aabb[1][1] > rear_rest_aabb[1][1] + 0.45,
            details=(
                f"rear max-y did not move forward enough: rest={rear_rest_aabb[1][1]:.3f}, "
                f"posed={rear_closed_aabb[1][1]:.3f}"
            ),
        )

    fly_rest_aabb = ctx.part_world_aabb(fly_section)
    assert fly_rest_aabb is not None
    with ctx.pose({fly_extension: 0.75}):
        fly_extended_aabb = ctx.part_world_aabb(fly_section)
        assert fly_extended_aabb is not None
        ctx.check(
            "fly_extension_raises_upper_section",
            fly_extended_aabb[1][2] > fly_rest_aabb[1][2] + 0.70,
            details=(
                f"fly top did not rise enough: rest={fly_rest_aabb[1][2]:.3f}, "
                f"posed={fly_extended_aabb[1][2]:.3f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
