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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _shift_profile(
    profile: list[tuple[float, float]],
    *,
    du: float = 0.0,
    dv: float = 0.0,
) -> list[tuple[float, float]]:
    return [(u + du, v + dv) for u, v in profile]


def _yz_profile_to_local_xy(profile_yz: list[tuple[float, float]]) -> list[tuple[float, float]]:
    # ExtrudeWithHolesGeometry builds a profile in local XY and extrudes along +Z.
    # Rotating the result by +90 deg about Y makes:
    #   local Z -> world X
    #   local Y -> world Y
    #   local X -> -world Z
    # So a desired (y, z) profile maps to local (-z, y).
    return [(-z, y) for y, z in profile_yz]


def _build_base_plate_mesh():
    outer = rounded_rect_profile(0.280, 0.120, 0.008, corner_segments=6)
    slot = rounded_rect_profile(0.086, 0.016, 0.006, corner_segments=6)
    geom = ExtrudeWithHolesGeometry(
        outer,
        [
            _shift_profile(slot, dv=0.042),
            _shift_profile(slot, dv=-0.042),
        ],
        height=0.012,
        center=True,
    )
    geom.translate(0.0, 0.0, 0.006)
    return geom


def _build_moving_jaw_mesh():
    outer_yz = [
        (-0.052, 0.012),
        (0.052, 0.012),
        (0.052, 0.040),
        (0.044, 0.066),
        (-0.044, 0.066),
        (-0.052, 0.040),
    ]
    square_slot = [
        (-0.010, -0.010),
        (0.010, -0.010),
        (0.010, 0.010),
        (-0.010, 0.010),
    ]
    center_slot = rounded_rect_profile(0.016, 0.018, 0.003, corner_segments=5)
    geom = ExtrudeWithHolesGeometry(
        _yz_profile_to_local_xy(outer_yz),
        [
            _yz_profile_to_local_xy(_shift_profile(square_slot, du=-0.032, dv=0.022)),
            _yz_profile_to_local_xy(_shift_profile(square_slot, du=0.032, dv=0.022)),
            _yz_profile_to_local_xy(_shift_profile(center_slot, du=0.000, dv=0.030)),
        ],
        height=0.028,
        center=True,
    )
    geom.rotate_y(pi / 2.0)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="quick_action_drill_press_vise")

    cast_iron = model.material("cast_iron", rgba=(0.29, 0.31, 0.34, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    lever_red = model.material("lever_red", rgba=(0.58, 0.10, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        _mesh("vise_base_plate", _build_base_plate_mesh()),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        Box((0.032, 0.104, 0.048)),
        origin=Origin(xyz=(-0.083, 0.0, 0.036)),
        material=cast_iron,
        name="fixed_jaw_body",
    )
    base.visual(
        Box((0.014, 0.018, 0.030)),
        origin=Origin(xyz=(-0.077, 0.043, 0.027)),
        material=cast_iron,
        name="fixed_jaw_right_gusset",
    )
    base.visual(
        Box((0.014, 0.018, 0.030)),
        origin=Origin(xyz=(-0.077, -0.043, 0.027)),
        material=cast_iron,
        name="fixed_jaw_left_gusset",
    )
    base.visual(
        Box((0.006, 0.092, 0.030)),
        origin=Origin(xyz=(-0.064, 0.0, 0.042)),
        material=dark_steel,
        name="fixed_jaw_face",
    )
    base.visual(
        Box((0.176, 0.018, 0.016)),
        origin=Origin(xyz=(0.018, 0.032, 0.020)),
        material=machined_steel,
        name="guide_bar_right",
    )
    base.visual(
        Box((0.176, 0.018, 0.016)),
        origin=Origin(xyz=(0.018, -0.032, 0.020)),
        material=machined_steel,
        name="guide_bar_left",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.280, 0.120, 0.080)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )

    moving_jaw = model.part("moving_jaw")
    moving_jaw.visual(
        Box((0.028, 0.104, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=cast_iron,
        name="carriage_body",
    )
    moving_jaw.visual(
        Box((0.028, 0.046, 0.034)),
        origin=Origin(xyz=(-0.002, 0.0, 0.029)),
        material=cast_iron,
        name="carriage_web",
    )
    moving_jaw.visual(
        Box((0.028, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.050, 0.034)),
        material=cast_iron,
        name="right_bar_cheek",
    )
    moving_jaw.visual(
        Box((0.028, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.050, 0.034)),
        material=cast_iron,
        name="left_bar_cheek",
    )
    moving_jaw.visual(
        Box((0.006, 0.092, 0.030)),
        origin=Origin(xyz=(-0.017, 0.0, 0.042)),
        material=dark_steel,
        name="jaw_face",
    )
    moving_jaw.visual(
        Box((0.012, 0.072, 0.010)),
        origin=Origin(xyz=(0.004, 0.0, 0.061)),
        material=cast_iron,
        name="jaw_crown",
    )
    moving_jaw.inertial = Inertial.from_geometry(
        Box((0.034, 0.104, 0.056)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
    )

    release_lever = model.part("release_lever")
    release_lever.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_hub",
    )
    release_lever.visual(
        Cylinder(radius=0.007, length=0.008),
        origin=Origin(xyz=(-0.004, 0.0, -0.006), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="cam_lobe",
    )
    release_lever.visual(
        Box((0.050, 0.010, 0.012)),
        origin=Origin(xyz=(0.025, 0.0, -0.006)),
        material=dark_steel,
        name="lever_arm",
    )
    release_lever.visual(
        Cylinder(radius=0.007, length=0.022),
        origin=Origin(xyz=(0.050, 0.0, -0.006), rpy=(pi / 2.0, 0.0, 0.0)),
        material=lever_red,
        name="lever_grip",
    )
    release_lever.inertial = Inertial.from_geometry(
        Box((0.066, 0.022, 0.026)),
        mass=0.20,
        origin=Origin(xyz=(0.022, 0.0, -0.006)),
    )

    screw_handle = model.part("screw_handle")
    screw_handle.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="spindle",
    )
    screw_handle.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.002, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined_steel,
        name="thrust_collar",
    )
    screw_handle.visual(
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined_steel,
        name="handle_hub",
    )
    screw_handle.visual(
        Cylinder(radius=0.0045, length=0.084),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="handle_bar",
    )
    screw_handle.visual(
        Sphere(radius=0.007),
        origin=Origin(xyz=(0.020, 0.042, 0.0)),
        material=dark_steel,
        name="handle_knob_right",
    )
    screw_handle.visual(
        Sphere(radius=0.007),
        origin=Origin(xyz=(0.020, -0.042, 0.0)),
        material=dark_steel,
        name="handle_knob_left",
    )
    screw_handle.inertial = Inertial.from_geometry(
        Box((0.040, 0.090, 0.016)),
        mass=0.30,
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_moving_jaw",
        ArticulationType.PRISMATIC,
        parent=base,
        child=moving_jaw,
        origin=Origin(xyz=(-0.039, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.12, lower=0.0, upper=0.096),
    )
    model.articulation(
        "moving_jaw_to_release_lever",
        ArticulationType.REVOLUTE,
        parent=moving_jaw,
        child=release_lever,
        origin=Origin(xyz=(-0.004, 0.064, 0.034)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.4, lower=0.0, upper=1.15),
    )
    model.articulation(
        "moving_jaw_to_screw_handle",
        ArticulationType.CONTINUOUS,
        parent=moving_jaw,
        child=screw_handle,
        origin=Origin(xyz=(0.014, 0.0, 0.046)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=10.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    moving_jaw = object_model.get_part("moving_jaw")
    release_lever = object_model.get_part("release_lever")
    screw_handle = object_model.get_part("screw_handle")
    jaw_slide = object_model.get_articulation("base_to_moving_jaw")
    lever_joint = object_model.get_articulation("moving_jaw_to_release_lever")
    handle_joint = object_model.get_articulation("moving_jaw_to_screw_handle")

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

    slide_upper = jaw_slide.motion_limits.upper if jaw_slide.motion_limits is not None else None
    lever_upper = lever_joint.motion_limits.upper if lever_joint.motion_limits is not None else None

    ctx.check(
        "jaw slide uses a forward prismatic axis",
        jaw_slide.articulation_type == ArticulationType.PRISMATIC and jaw_slide.axis == (1.0, 0.0, 0.0),
        details=f"type={jaw_slide.articulation_type}, axis={jaw_slide.axis}",
    )
    ctx.check(
        "release lever pivots on the jaw side",
        lever_joint.articulation_type == ArticulationType.REVOLUTE and lever_joint.axis == (0.0, -1.0, 0.0),
        details=f"type={lever_joint.articulation_type}, axis={lever_joint.axis}",
    )
    ctx.check(
        "fine adjust handle spins on the screw axis",
        handle_joint.articulation_type == ArticulationType.CONTINUOUS and handle_joint.axis == (1.0, 0.0, 0.0),
        details=f"type={handle_joint.articulation_type}, axis={handle_joint.axis}",
    )

    with ctx.pose({jaw_slide: 0.0}):
        ctx.expect_gap(
            moving_jaw,
            base,
            axis="x",
            positive_elem="jaw_face",
            negative_elem="fixed_jaw_face",
            min_gap=0.001,
            max_gap=0.0035,
            name="closed jaws leave a fine clamping gap",
        )
        ctx.expect_overlap(
            moving_jaw,
            base,
            axes="yz",
            elem_a="jaw_face",
            elem_b="fixed_jaw_face",
            min_overlap=0.020,
            name="jaw faces align across width and height",
        )
        ctx.expect_contact(
            moving_jaw,
            base,
            elem_a="carriage_web",
            elem_b="guide_bar_right",
            contact_tol=5e-4,
            name="moving jaw engages the right guide bar",
        )
        ctx.expect_contact(
            moving_jaw,
            base,
            elem_a="carriage_web",
            elem_b="guide_bar_left",
            contact_tol=5e-4,
            name="moving jaw engages the left guide bar",
        )
        ctx.expect_contact(
            release_lever,
            moving_jaw,
            elem_a="pivot_hub",
            elem_b="right_bar_cheek",
            contact_tol=5e-4,
            name="release lever is mounted on the moving jaw",
        )
        ctx.expect_contact(
            screw_handle,
            moving_jaw,
            elem_a="thrust_collar",
            elem_b="carriage_body",
            contact_tol=5e-4,
            name="screw handle is retained by a front collar",
        )

    rest_jaw_pos = ctx.part_world_position(moving_jaw)
    if slide_upper is not None:
        with ctx.pose({jaw_slide: slide_upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps at maximum jaw opening")
            ctx.expect_contact(
                moving_jaw,
                base,
                elem_a="carriage_web",
                elem_b="guide_bar_right",
                contact_tol=5e-4,
                name="moving jaw stays engaged on the guide bars when fully opened",
            )
            ctx.expect_overlap(
                moving_jaw,
                base,
                axes="x",
                elem_a="carriage_web",
                elem_b="guide_bar_right",
                min_overlap=0.020,
                name="right guide bar remains engaged at full opening",
            )
            open_jaw_pos = ctx.part_world_position(moving_jaw)
        ctx.check(
            "moving jaw opens forward",
            rest_jaw_pos is not None and open_jaw_pos is not None and open_jaw_pos[0] > rest_jaw_pos[0] + 0.080,
            details=f"rest={rest_jaw_pos}, open={open_jaw_pos}",
        )

    if lever_upper is not None:
        lever_rest = ctx.part_element_world_aabb(release_lever, elem="lever_grip")
        with ctx.pose({lever_joint: lever_upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps with quick release lever opened")
            lever_open = ctx.part_element_world_aabb(release_lever, elem="lever_grip")
        rest_z = None if lever_rest is None else (lever_rest[0][2] + lever_rest[1][2]) * 0.5
        open_z = None if lever_open is None else (lever_open[0][2] + lever_open[1][2]) * 0.5
        ctx.check(
            "release lever lifts upward when opened",
            rest_z is not None and open_z is not None and open_z > rest_z + 0.015,
            details=f"rest_z={rest_z}, open_z={open_z}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
