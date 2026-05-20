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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="appliance_hand_truck")

    frame_blue = model.material("frame_blue", rgba=(0.13, 0.31, 0.68, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.75, 0.78, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
        return [(-x, y, z) for x, y, z in points]

    def add_wheel_visuals(part, prefix: str, *, radius: float, width: float, inboard_sign: float) -> None:
        half_width = width * 0.5
        spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
        tire_profile = [
            (radius * 0.62, -half_width * 0.96),
            (radius * 0.78, -half_width),
            (radius * 0.92, -half_width * 0.84),
            (radius * 0.99, -half_width * 0.52),
            (radius, -half_width * 0.14),
            (radius, half_width * 0.14),
            (radius * 0.99, half_width * 0.52),
            (radius * 0.92, half_width * 0.84),
            (radius * 0.78, half_width),
            (radius * 0.62, half_width * 0.96),
            (radius * 0.49, half_width * 0.28),
            (radius * 0.45, 0.0),
            (radius * 0.49, -half_width * 0.28),
            (radius * 0.62, -half_width * 0.96),
        ]
        tire = save_mesh(f"{prefix}_tire", LatheGeometry(tire_profile, segments=56).rotate_y(pi / 2.0))
        part.visual(tire, material=rubber, name="tire")

        rim_radius = radius * 0.72
        part.visual(
            Cylinder(radius=rim_radius * 0.96, length=width * 0.10),
            origin=Origin(xyz=(width * 0.16, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=aluminum,
            name="outer_rim",
        )
        part.visual(
            Cylinder(radius=rim_radius * 0.96, length=width * 0.10),
            origin=Origin(xyz=(-width * 0.16, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=aluminum,
            name="inner_rim",
        )
        part.visual(
            Cylinder(radius=radius * 0.30, length=width * 0.74),
            origin=spin_origin,
            material=dark_steel,
            name="hub",
        )
        part.visual(
            Cylinder(radius=radius * 0.14, length=width * 0.22),
            origin=Origin(xyz=(width * 0.18, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=aluminum,
            name="hub_cap",
        )
        part.visual(
            Cylinder(radius=radius * 0.16, length=0.020),
            origin=Origin(xyz=(0.020 * inboard_sign, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name="inboard_bushing",
        )
        part.visual(
            Box((width * 0.12, 0.016, 0.028)),
            origin=Origin(xyz=(width * 0.18, 0.0, rim_radius * 0.84)),
            material=aluminum,
            name="rim_marker",
        )

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.56, 0.18, 1.36)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
    )

    left_upright_points = [
        (0.20, 0.0, 0.07),
        (0.20, 0.0, 0.56),
        (0.20, 0.0, 1.02),
        (0.18, 0.03, 1.24),
        (0.15, 0.05, 1.30),
    ]
    frame.visual(
        save_mesh(
            "frame_left_upright",
            tube_from_spline_points(left_upright_points, radius=0.022, samples_per_segment=16, radial_segments=18),
        ),
        material=frame_blue,
        name="left_upright",
    )
    frame.visual(
        save_mesh(
            "frame_right_upright",
            tube_from_spline_points(mirror_x(left_upright_points), radius=0.022, samples_per_segment=16, radial_segments=18),
        ),
        material=frame_blue,
        name="right_upright",
    )
    frame.visual(
        save_mesh(
            "frame_top_handle",
            tube_from_spline_points(
                [(-0.15, 0.05, 1.30), (0.0, 0.085, 1.33), (0.15, 0.05, 1.30)],
                radius=0.022,
                samples_per_segment=20,
                radial_segments=18,
            ),
        ),
        material=frame_blue,
        name="top_handle",
    )
    frame.visual(
        Box((0.42, 0.046, 0.07)),
        origin=Origin(xyz=(0.0, 0.003, 0.035)),
        material=frame_blue,
        name="frame_base_housing",
    )
    frame.visual(
        Box((0.06, 0.06, 0.11)),
        origin=Origin(xyz=(0.225, -0.03, 0.12)),
        material=frame_blue,
        name="left_wheel_fork",
    )
    frame.visual(
        Box((0.06, 0.06, 0.11)),
        origin=Origin(xyz=(-0.225, -0.03, 0.12)),
        material=frame_blue,
        name="right_wheel_fork",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.44),
        origin=Origin(xyz=(0.0, -0.03, 0.145), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="axle_tube",
    )
    frame.visual(
        save_mesh(
            "frame_left_lower_brace",
            tube_from_spline_points(
                [(0.20, 0.0, 0.30), (0.215, -0.015, 0.24), (0.225, -0.03, 0.17)],
                radius=0.016,
                samples_per_segment=10,
                radial_segments=16,
            ),
        ),
        material=frame_blue,
        name="left_lower_brace",
    )
    frame.visual(
        save_mesh(
            "frame_right_lower_brace",
            tube_from_spline_points(
                [(-0.20, 0.0, 0.30), (-0.215, -0.015, 0.24), (-0.225, -0.03, 0.17)],
                radius=0.016,
                samples_per_segment=10,
                radial_segments=16,
            ),
        ),
        material=frame_blue,
        name="right_lower_brace",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 0.38), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_blue,
        name="lower_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 0.68), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_blue,
        name="mid_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 0.98), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_blue,
        name="upper_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(0.185, 0.05, 1.30), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(-0.185, 0.05, 1.30), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )

    toe_plate = model.part("toe_plate")
    toe_plate.inertial = Inertial.from_geometry(
        Box((0.48, 0.32, 0.06)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.17, 0.02)),
    )
    toe_plate.visual(
        Box((0.46, 0.28, 0.012)),
        origin=Origin(xyz=(0.0, 0.17, -0.034)),
        material=aluminum,
        name="toe_panel",
    )
    toe_plate.visual(
        Box((0.46, 0.016, 0.042)),
        origin=Origin(xyz=(0.0, 0.302, -0.019)),
        material=aluminum,
        name="toe_front_lip",
    )
    toe_plate.visual(
        Box((0.014, 0.28, 0.042)),
        origin=Origin(xyz=(0.223, 0.17, -0.019)),
        material=aluminum,
        name="left_side_flange",
    )
    toe_plate.visual(
        Box((0.014, 0.28, 0.042)),
        origin=Origin(xyz=(-0.223, 0.17, -0.019)),
        material=aluminum,
        name="right_side_flange",
    )
    toe_plate.visual(
        Box((0.26, 0.035, 0.028)),
        origin=Origin(xyz=(0.0, 0.07, -0.023)),
        material=frame_blue,
        name="toe_rear_stiffener",
    )
    toe_plate.visual(
        Box((0.10, 0.07, 0.05)),
        origin=Origin(xyz=(0.125, 0.045, -0.014)),
        material=frame_blue,
        name="left_hinge_arm",
    )
    toe_plate.visual(
        Box((0.10, 0.07, 0.05)),
        origin=Origin(xyz=(-0.125, 0.045, -0.014)),
        material=frame_blue,
        name="right_hinge_arm",
    )
    toe_plate.visual(
        Cylinder(radius=0.014, length=0.07),
        origin=Origin(xyz=(0.115, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_hinge_barrel",
    )
    toe_plate.visual(
        Cylinder(radius=0.014, length=0.07),
        origin=Origin(xyz=(-0.115, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_hinge_barrel",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.145, length=0.055),
        mass=3.2,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    add_wheel_visuals(left_wheel, "left_wheel", radius=0.145, width=0.055, inboard_sign=-1.0)

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.145, length=0.055),
        mass=3.2,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    add_wheel_visuals(right_wheel, "right_wheel", radius=0.145, width=0.055, inboard_sign=1.0)

    toe_hinge = model.articulation(
        "frame_to_toe_plate",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=toe_plate,
        origin=Origin(xyz=(0.0, 0.04, 0.04)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=0.0, upper=1.52),
    )
    model.articulation(
        "frame_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_wheel,
        origin=Origin(xyz=(0.285, -0.03, 0.145)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=22.0),
    )
    model.articulation(
        "frame_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_wheel,
        origin=Origin(xyz=(-0.285, -0.03, 0.145)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=22.0),
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

    frame = object_model.get_part("frame")
    toe_plate = object_model.get_part("toe_plate")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")

    toe_hinge = object_model.get_articulation("frame_to_toe_plate")
    left_spin = object_model.get_articulation("frame_to_left_wheel")
    right_spin = object_model.get_articulation("frame_to_right_wheel")

    def aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    def aabb_span(aabb):
        return tuple(aabb[1][i] - aabb[0][i] for i in range(3))

    ctx.check(
        "major parts and joints exist",
        all(obj is not None for obj in (frame, toe_plate, left_wheel, right_wheel, toe_hinge, left_spin, right_spin)),
    )
    ctx.check(
        "toe plate hinge uses a transverse axle",
        toe_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(toe_hinge.axis) == (1.0, 0.0, 0.0)
        and toe_hinge.motion_limits is not None
        and toe_hinge.motion_limits.lower == 0.0
        and toe_hinge.motion_limits.upper is not None
        and toe_hinge.motion_limits.upper > 1.4,
        details=f"type={toe_hinge.articulation_type}, axis={toe_hinge.axis}, limits={toe_hinge.motion_limits}",
    )
    ctx.check(
        "rear wheels spin continuously on the axle",
        left_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_spin.axis) == (1.0, 0.0, 0.0)
        and tuple(right_spin.axis) == (1.0, 0.0, 0.0),
        details=f"left={left_spin.axis}, right={right_spin.axis}",
    )

    ctx.expect_origin_distance(
        left_wheel,
        right_wheel,
        axes="x",
        min_dist=0.55,
        max_dist=0.59,
        name="wheel centers sit at a realistic axle width",
    )

    with ctx.pose({toe_hinge: 0.0}):
        ctx.expect_gap(
            toe_plate,
            frame,
            axis="y",
            min_gap=0.008,
            positive_elem="toe_panel",
            negative_elem="frame_base_housing",
            name="open toe plate projects ahead of the lower frame",
        )
        ctx.expect_overlap(
            toe_plate,
            frame,
            axes="x",
            min_overlap=0.40,
            elem_a="toe_panel",
            elem_b="frame_base_housing",
            name="toe plate is broad across the lower frame width",
        )
        open_panel_aabb = ctx.part_element_world_aabb(toe_plate, elem="toe_panel")

    folded_angle = toe_hinge.motion_limits.upper if toe_hinge.motion_limits is not None else 1.52
    with ctx.pose({toe_hinge: folded_angle}):
        folded_panel_aabb = ctx.part_element_world_aabb(toe_plate, elem="toe_panel")
        ctx.expect_gap(
            toe_plate,
            frame,
            axis="y",
            min_gap=0.0,
            max_gap=0.08,
            positive_elem="toe_panel",
            negative_elem="frame_base_housing",
            name="folded toe plate tucks close to the frame front",
        )

    if open_panel_aabb is not None and folded_panel_aabb is not None:
        open_center = aabb_center(open_panel_aabb)
        folded_center = aabb_center(folded_panel_aabb)
        open_span = aabb_span(open_panel_aabb)
        folded_span = aabb_span(folded_panel_aabb)
        ctx.check(
            "toe plate folds from horizontal loading shelf to near-vertical stowed pose",
            open_span[1] > 0.25
            and open_span[2] < 0.03
            and folded_span[2] > 0.25
            and folded_span[1] < 0.05
            and folded_center[2] > open_center[2] + 0.12,
            details=(
                f"open_center={open_center}, folded_center={folded_center}, "
                f"open_span={open_span}, folded_span={folded_span}"
            ),
        )

    left_marker_rest = ctx.part_element_world_aabb(left_wheel, elem="rim_marker")
    with ctx.pose({left_spin: pi / 2.0}):
        left_marker_quarter_turn = ctx.part_element_world_aabb(left_wheel, elem="rim_marker")

    if left_marker_rest is not None and left_marker_quarter_turn is not None:
        rest_center = aabb_center(left_marker_rest)
        turned_center = aabb_center(left_marker_quarter_turn)
        ctx.check(
            "left wheel quarter turn moves a rim feature around the axle axis",
            abs(turned_center[1] - rest_center[1]) > 0.07
            and rest_center[2] > turned_center[2] + 0.07,
            details=f"rest_center={rest_center}, turned_center={turned_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
