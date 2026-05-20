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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _build_frame_cheek_mesh(
    *,
    profile: list[tuple[float, float]],
    plate_thickness: float,
    y_center: float,
    mesh_name: str,
):
    cheek = ExtrudeGeometry.centered(profile, plate_thickness, cap=True, closed=True)
    cheek.rotate_x(math.pi / 2.0)
    cheek.translate(0.0, y_center, 0.0)
    return mesh_from_geometry(cheek, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_c_clamp_vise")

    frame_steel = model.material("frame_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    screw_steel = model.material("screw_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.10, 0.11, 0.12, 1.0))

    plate_thickness = 0.012
    frame_gap = 0.032
    frame_depth = frame_gap + (2.0 * plate_thickness)
    cheek_y = (frame_gap * 0.5) + (plate_thickness * 0.5)

    cheek_profile = [
        (-0.100, -0.140),
        (0.030, -0.140),
        (0.050, -0.132),
        (0.066, -0.116),
        (0.074, -0.095),
        (0.074, -0.074),
        (-0.044, -0.074),
        (-0.044, 0.070),
        (0.086, 0.070),
        (0.098, 0.082),
        (0.098, 0.140),
        (-0.100, 0.140),
    ]

    frame = model.part("frame")
    frame.visual(
        _build_frame_cheek_mesh(
            profile=cheek_profile,
            plate_thickness=plate_thickness,
            y_center=cheek_y,
            mesh_name="frame_left_cheek",
        ),
        material=frame_steel,
        name="left_cheek",
    )
    frame.visual(
        _build_frame_cheek_mesh(
            profile=cheek_profile,
            plate_thickness=plate_thickness,
            y_center=-cheek_y,
            mesh_name="frame_right_cheek",
        ),
        material=frame_steel,
        name="right_cheek",
    )
    frame.visual(
        Box((0.056, frame_depth, 0.220)),
        origin=Origin(xyz=(-0.072, 0.0, 0.000)),
        material=frame_steel,
        name="rear_spine",
    )
    frame.visual(
        Box((0.142, frame_depth, 0.056)),
        origin=Origin(xyz=(0.016, 0.0, 0.112)),
        material=frame_steel,
        name="upper_arm",
    )
    frame.visual(
        Box((0.086, frame_depth, 0.050)),
        origin=Origin(xyz=(-0.042, 0.0, -0.109)),
        material=frame_steel,
        name="lower_arm_bridge",
    )
    frame.visual(
        Box((0.032, frame_depth, 0.020)),
        origin=Origin(xyz=(0.076, 0.0, 0.082)),
        material=frame_steel,
        name="fixed_jaw_face",
    )

    lower_jaw = model.part("lower_jaw_assembly")
    lower_jaw.visual(
        Cylinder(radius=0.009, length=0.117),
        origin=Origin(xyz=(0.0, 0.0, 0.0585)),
        material=screw_steel,
        name="screw_shaft",
    )
    lower_jaw.visual(
        Cylinder(radius=0.0175, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=screw_steel,
        name="screw_head",
    )
    lower_jaw.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=screw_steel,
        name="pad_stem",
    )
    lower_jaw.visual(
        Box((0.054, 0.044, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.113)),
        material=screw_steel,
        name="jaw_pad",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=handle_finish,
        name="handle_hub",
    )
    handle.visual(
        Cylinder(radius=0.0045, length=0.114),
        origin=Origin(
            xyz=(0.0, 0.0, -0.005),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=handle_finish,
        name="handle_bar",
    )
    for side, x_sign in (("left", -1.0), ("right", 1.0)):
        handle.visual(
            Cylinder(radius=0.0065, length=0.024),
            origin=Origin(
                xyz=(x_sign * 0.045, 0.0, -0.005),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=handle_finish,
            name=f"{side}_handle_grip",
        )

    jaw_slide = model.articulation(
        "frame_to_lower_jaw",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=lower_jaw,
        origin=Origin(xyz=(0.072, 0.0, -0.105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.030,
            lower=0.0,
            upper=0.050,
        ),
    )
    model.articulation(
        "lower_jaw_to_handle",
        ArticulationType.CONTINUOUS,
        parent=lower_jaw,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=10.0,
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

    frame = object_model.get_part("frame")
    lower_jaw = object_model.get_part("lower_jaw_assembly")
    handle = object_model.get_part("handle")
    jaw_slide = object_model.get_articulation("frame_to_lower_jaw")
    handle_spin = object_model.get_articulation("lower_jaw_to_handle")

    ctx.check(
        "jaw slide is vertical prismatic",
        jaw_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(jaw_slide.axis) == (0.0, 0.0, 1.0)
        and jaw_slide.motion_limits is not None
        and jaw_slide.motion_limits.lower == 0.0
        and jaw_slide.motion_limits.upper is not None
        and jaw_slide.motion_limits.upper > 0.045,
        details=(
            f"type={jaw_slide.articulation_type}, axis={jaw_slide.axis}, "
            f"limits={jaw_slide.motion_limits}"
        ),
    )
    ctx.check(
        "handle spins continuously",
        handle_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(handle_spin.axis) == (0.0, 0.0, 1.0)
        and handle_spin.motion_limits is not None
        and handle_spin.motion_limits.lower is None
        and handle_spin.motion_limits.upper is None,
        details=(
            f"type={handle_spin.articulation_type}, axis={handle_spin.axis}, "
            f"limits={handle_spin.motion_limits}"
        ),
    )

    with ctx.pose({jaw_slide: 0.0}):
        ctx.expect_overlap(
            frame,
            lower_jaw,
            axes="xy",
            elem_a="fixed_jaw_face",
            elem_b="jaw_pad",
            min_overlap=0.020,
            name="open pad stays under the fixed jaw footprint",
        )
        ctx.expect_gap(
            frame,
            lower_jaw,
            axis="z",
            positive_elem="fixed_jaw_face",
            negative_elem="jaw_pad",
            min_gap=0.045,
            max_gap=0.060,
            name="open clamp leaves usable jaw opening",
        )

    slide_upper = jaw_slide.motion_limits.upper if jaw_slide.motion_limits is not None else None
    if slide_upper is not None:
        rest_pos = ctx.part_world_position(lower_jaw)
        with ctx.pose({jaw_slide: slide_upper}):
            extended_pos = ctx.part_world_position(lower_jaw)
            ctx.expect_overlap(
                frame,
                lower_jaw,
                axes="xy",
                elem_a="fixed_jaw_face",
                elem_b="jaw_pad",
                min_overlap=0.020,
                name="closed pad remains aligned with fixed jaw",
            )
            ctx.expect_gap(
                frame,
                lower_jaw,
                axis="z",
                positive_elem="fixed_jaw_face",
                negative_elem="jaw_pad",
                min_gap=0.0,
                max_gap=0.008,
                max_penetration=0.0,
                name="closed clamp nearly seats pad to fixed jaw",
            )
        ctx.check(
            "lower jaw travels upward when tightened",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[2] > rest_pos[2] + 0.040,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    handle_rest_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_spin: math.pi / 2.0}):
        ctx.expect_contact(
            lower_jaw,
            handle,
            elem_a="screw_head",
            elem_b="handle_hub",
            contact_tol=0.0005,
            name="handle hub stays seated against the screw head while spinning",
        )
        handle_turned_aabb = ctx.part_world_aabb(handle)

    ctx.check(
        "handle quarter-turn changes crossbar orientation",
        handle_rest_aabb is not None
        and handle_turned_aabb is not None
        and (handle_rest_aabb[1][0] - handle_rest_aabb[0][0])
        > (handle_rest_aabb[1][1] - handle_rest_aabb[0][1])
        and (handle_turned_aabb[1][1] - handle_turned_aabb[0][1])
        > (handle_turned_aabb[1][0] - handle_turned_aabb[0][0]),
        details=f"rest_aabb={handle_rest_aabb}, turned_aabb={handle_turned_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
