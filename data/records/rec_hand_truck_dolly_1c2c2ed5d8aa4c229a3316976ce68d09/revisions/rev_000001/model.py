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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_luggage_hand_truck")

    aluminum = model.material("aluminum", rgba=(0.80, 0.82, 0.84, 1.0))
    darker_aluminum = model.material("darker_aluminum", rgba=(0.62, 0.66, 0.70, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.07, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.10, 1.0))

    def rect_profile(width: float, depth: float) -> list[tuple[float, float]]:
        hw = width * 0.5
        hd = depth * 0.5
        return [(-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd)]

    sleeve_shell = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rect_profile(0.032, 0.024),
            [rect_profile(0.022, 0.014)],
            height=0.32,
            center=True,
        ),
        "hand_truck_outer_sleeve_shell",
    )
    sleeve_collar_shell = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rect_profile(0.040, 0.028),
            [rect_profile(0.032, 0.024)],
            height=0.026,
            center=True,
        ),
        "hand_truck_sleeve_collar_shell",
    )

    frame = model.part("frame")
    frame.visual(
        Box((0.028, 0.020, 0.38)),
        origin=Origin(xyz=(-0.145, 0.012, 0.25)),
        material=aluminum,
        name="left_side_rail",
    )
    frame.visual(
        Box((0.028, 0.020, 0.38)),
        origin=Origin(xyz=(0.145, 0.012, 0.25)),
        material=aluminum,
        name="right_side_rail",
    )
    frame.visual(
        sleeve_shell,
        origin=Origin(xyz=(-0.145, 0.012, 0.60)),
        material=aluminum,
        name="left_outer_sleeve",
    )
    frame.visual(
        sleeve_shell,
        origin=Origin(xyz=(0.145, 0.012, 0.60)),
        material=aluminum,
        name="right_outer_sleeve",
    )
    frame.visual(
        sleeve_collar_shell,
        origin=Origin(xyz=(-0.145, 0.012, 0.447)),
        material=darker_aluminum,
        name="left_sleeve_collar",
    )
    frame.visual(
        sleeve_collar_shell,
        origin=Origin(xyz=(0.145, 0.012, 0.447)),
        material=darker_aluminum,
        name="right_sleeve_collar",
    )
    frame.visual(
        Box((0.31, 0.024, 0.034)),
        origin=Origin(xyz=(0.0, 0.017, 0.058)),
        material=darker_aluminum,
        name="lower_front_bar",
    )
    frame.visual(
        Box((0.30, 0.060, 0.048)),
        origin=Origin(xyz=(0.0, -0.010, 0.104)),
        material=charcoal,
        name="axle_truss",
    )
    frame.visual(
        Box((0.30, 0.024, 0.032)),
        origin=Origin(xyz=(0.0, 0.006, 0.28)),
        material=darker_aluminum,
        name="mid_cross_brace",
    )
    frame.visual(
        Box((0.30, 0.008, 0.024)),
        origin=Origin(xyz=(0.0, -0.004, 0.461)),
        material=darker_aluminum,
        name="upper_cross_brace",
    )
    frame.visual(
        Box((0.21, 0.010, 0.24)),
        origin=Origin(xyz=(0.0, -0.002, 0.33)),
        material=charcoal,
        name="back_panel",
    )
    frame.visual(
        Box((0.012, 0.094, 0.032)),
        origin=Origin(xyz=(-0.151, -0.035, 0.100)),
        material=darker_aluminum,
        name="left_wheel_strut",
    )
    frame.visual(
        Box((0.012, 0.094, 0.032)),
        origin=Origin(xyz=(0.151, -0.035, 0.100)),
        material=darker_aluminum,
        name="right_wheel_strut",
    )
    frame.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(-0.157, -0.082, 0.095), rpy=(0.0, pi / 2.0, 0.0)),
        material=charcoal,
        name="left_spindle",
    )
    frame.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.157, -0.082, 0.095), rpy=(0.0, pi / 2.0, 0.0)),
        material=charcoal,
        name="right_spindle",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.38, 0.15, 0.78)),
        mass=4.8,
        origin=Origin(xyz=(0.0, -0.01, 0.39)),
    )

    handle_assembly = model.part("handle_assembly")
    handle_assembly.visual(
        Box((0.018, 0.010, 0.56)),
        origin=Origin(xyz=(-0.145, 0.012, 0.28)),
        material=aluminum,
        name="left_inner_tube",
    )
    handle_assembly.visual(
        Box((0.018, 0.010, 0.56)),
        origin=Origin(xyz=(0.145, 0.012, 0.28)),
        material=aluminum,
        name="right_inner_tube",
    )
    handle_assembly.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.145, 0.012, 0.54),
                    (-0.132, 0.030, 0.58),
                    (-0.060, 0.058, 0.62),
                    (0.060, 0.058, 0.62),
                    (0.132, 0.030, 0.58),
                    (0.145, 0.012, 0.54),
                ],
                radius=0.009,
                samples_per_segment=16,
                radial_segments=18,
                cap_ends=True,
            ),
            "hand_truck_top_bail",
        ),
        material=aluminum,
        name="top_bail",
    )
    handle_assembly.visual(
        Cylinder(radius=0.014, length=0.18),
        origin=Origin(xyz=(0.0, 0.058, 0.62), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_black,
        name="foam_grip",
    )
    handle_assembly.visual(
        Box((0.024, 0.014, 0.020)),
        origin=Origin(xyz=(-0.145, 0.012, 0.340)),
        material=charcoal,
        name="left_lock_collar",
    )
    handle_assembly.visual(
        Box((0.024, 0.014, 0.020)),
        origin=Origin(xyz=(0.145, 0.012, 0.340)),
        material=charcoal,
        name="right_lock_collar",
    )
    handle_assembly.inertial = Inertial.from_geometry(
        Box((0.34, 0.12, 0.66)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.03, 0.33)),
    )

    toe_plate = model.part("toe_plate")
    toe_plate.visual(
        Cylinder(radius=0.008, length=0.14),
        origin=Origin(xyz=(0.0, 0.004, 0.004), rpy=(0.0, pi / 2.0, 0.0)),
        material=darker_aluminum,
        name="hinge_tube",
    )
    toe_plate.visual(
        Box((0.32, 0.008, 0.19)),
        origin=Origin(xyz=(0.0, 0.012, 0.095)),
        material=aluminum,
        name="main_plate",
    )
    toe_plate.visual(
        Box((0.32, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, 0.020, 0.181)),
        material=darker_aluminum,
        name="toe_lip",
    )
    toe_plate.inertial = Inertial.from_geometry(
        Box((0.32, 0.028, 0.20)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.016, 0.10)),
    )

    def add_wheel(part_name: str) -> None:
        wheel = model.part(part_name)
        wheel.visual(
            Cylinder(radius=0.082, length=0.034),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.060, length=0.022),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=charcoal,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.020, length=0.038),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=darker_aluminum,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.012, length=0.012),
            origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=aluminum,
            name="outer_cap",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.082, length=0.034),
            mass=0.55,
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        )

    add_wheel("left_wheel")
    add_wheel("right_wheel")

    model.articulation(
        "frame_to_handle",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=handle_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=0.22),
    )
    model.articulation(
        "frame_to_toe_plate",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=toe_plate,
        origin=Origin(xyz=(0.0, 0.029, 0.033)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.6, lower=0.0, upper=1.47),
    )
    model.articulation(
        "frame_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child="left_wheel",
        origin=Origin(xyz=(-0.177, -0.082, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )
    model.articulation(
        "frame_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child="right_wheel",
        origin=Origin(xyz=(0.177, -0.082, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
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
    handle = object_model.get_part("handle_assembly")
    toe_plate = object_model.get_part("toe_plate")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")

    handle_slide = object_model.get_articulation("frame_to_handle")
    toe_hinge = object_model.get_articulation("frame_to_toe_plate")
    left_spin = object_model.get_articulation("frame_to_left_wheel")
    right_spin = object_model.get_articulation("frame_to_right_wheel")

    slide_upper = handle_slide.motion_limits.upper or 0.0
    toe_upper = toe_hinge.motion_limits.upper or 0.0

    ctx.expect_overlap(
        handle,
        frame,
        axes="z",
        elem_a="left_inner_tube",
        elem_b="left_outer_sleeve",
        min_overlap=0.20,
        name="left handle tube remains inserted when collapsed",
    )
    ctx.expect_overlap(
        handle,
        frame,
        axes="z",
        elem_a="right_inner_tube",
        elem_b="right_outer_sleeve",
        min_overlap=0.20,
        name="right handle tube remains inserted when collapsed",
    )

    rest_handle_pos = ctx.part_world_position(handle)
    with ctx.pose({handle_slide: slide_upper}):
        ctx.expect_overlap(
            handle,
            frame,
            axes="z",
            elem_a="left_inner_tube",
            elem_b="left_outer_sleeve",
            min_overlap=0.09,
            name="left handle tube keeps retained insertion when extended",
        )
        ctx.expect_overlap(
            handle,
            frame,
            axes="z",
            elem_a="right_inner_tube",
            elem_b="right_outer_sleeve",
            min_overlap=0.09,
            name="right handle tube keeps retained insertion when extended",
        )
        extended_handle_pos = ctx.part_world_position(handle)

    ctx.check(
        "handle rises upward on positive slide travel",
        rest_handle_pos is not None
        and extended_handle_pos is not None
        and extended_handle_pos[2] > rest_handle_pos[2] + 0.18,
        details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
    )

    with ctx.pose({toe_hinge: 0.0}):
        closed_toe_aabb = ctx.part_world_aabb(toe_plate)
    with ctx.pose({toe_hinge: toe_upper}):
        open_toe_aabb = ctx.part_world_aabb(toe_plate)

    ctx.check(
        "toe plate opens forward from the frame",
        closed_toe_aabb is not None
        and open_toe_aabb is not None
        and open_toe_aabb[1][1] > closed_toe_aabb[1][1] + 0.14,
        details=f"closed={closed_toe_aabb}, open={open_toe_aabb}",
    )

    ctx.check(
        "toe plate hinge uses the axle-width hinge axis",
        tuple(toe_hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"axis={toe_hinge.axis}",
    )
    ctx.check(
        "wheels use continuous spin joints on the axle axis",
        left_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_spin.axis) == (1.0, 0.0, 0.0)
        and tuple(right_spin.axis) == (1.0, 0.0, 0.0),
        details=(
            f"left=({left_spin.articulation_type}, {left_spin.axis}), "
            f"right=({right_spin.articulation_type}, {right_spin.axis})"
        ),
    )
    ctx.check(
        "wheels are mounted as a left-right pair",
        left_wheel is not None and right_wheel is not None,
        details="wheel parts must exist as separate rotating assemblies",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
