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
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(x_pos: float, width_y: float, height_z: float, radius: float) -> list[tuple[float, float, float]]:
    return [(x_pos, y, z) for y, z in rounded_rect_profile(width_y, height_z, radius, corner_segments=8)]


def _add_yoke_geometry(part, *, mesh_name: str, frame_material, metal_material) -> None:
    stem = tube_from_spline_points(
        [
            (-0.003, 0.0, -0.002),
            (-0.0035, 0.0, -0.016),
            (-0.004, 0.0, -0.030),
        ],
        radius=0.0042,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )
    front_arm = tube_from_spline_points(
        [
            (-0.004, 0.0, -0.030),
            (-0.0046, 0.010, -0.040),
            (-0.0046, 0.022, -0.052),
            (-0.004, 0.0265, -0.062),
        ],
        radius=0.0036,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )
    rear_arm = tube_from_spline_points(
        [
            (-0.004, 0.0, -0.030),
            (-0.0046, -0.010, -0.040),
            (-0.0046, -0.022, -0.052),
            (-0.004, -0.0265, -0.062),
        ],
        radius=0.0036,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )
    stem.merge(front_arm)
    stem.merge(rear_arm)

    part.visual(_mesh(mesh_name, stem), material=frame_material, name="yoke_frame")
    part.visual(
        Box((0.014, 0.016, 0.010)),
        origin=Origin(xyz=(-0.001, 0.0, -0.005)),
        material=frame_material,
        name="top_mount",
    )
    for sign, name in ((1.0, "front_pivot_pad"), (-1.0, "rear_pivot_pad")):
        part.visual(
            Cylinder(radius=0.007, length=0.004),
            origin=Origin(
                xyz=(-0.002, sign * 0.0265, -0.062),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=metal_material,
            name=name,
        )


def _add_earcup_geometry(
    part,
    *,
    shell_mesh_name: str,
    frame_mesh_name: str,
    cushion_mesh_name: str,
    shell_material,
    cushion_material,
    trim_material,
    with_slider_slot: bool = False,
) -> None:
    shell_cap = section_loft(
        [
            _yz_section(0.002, 0.070, 0.090, 0.020),
            _yz_section(0.010, 0.076, 0.096, 0.024),
            _yz_section(0.018, 0.074, 0.094, 0.023),
            _yz_section(0.024, 0.066, 0.084, 0.020),
        ]
    )
    part.visual(_mesh(shell_mesh_name, shell_cap), material=shell_material, name="shell_cap")

    outer_frame = rounded_rect_profile(0.076, 0.096, 0.024, corner_segments=8)
    inner_opening = rounded_rect_profile(0.050, 0.072, 0.017, corner_segments=8)
    frame_ring = ExtrudeWithHolesGeometry(
        outer_frame,
        [inner_opening],
        height=0.012,
        center=True,
    ).rotate_y(math.pi / 2.0).translate(0.028, 0.0, 0.0)
    part.visual(_mesh(frame_mesh_name, frame_ring), material=trim_material, name="bezel_frame")

    cushion_outer = rounded_rect_profile(0.086, 0.106, 0.028, corner_segments=8)
    cushion_inner = rounded_rect_profile(0.052, 0.074, 0.020, corner_segments=8)
    cushion_ring = ExtrudeWithHolesGeometry(
        cushion_outer,
        [cushion_inner],
        height=0.018,
        center=True,
    ).rotate_y(math.pi / 2.0).translate(0.043, 0.0, 0.0)
    part.visual(_mesh(cushion_mesh_name, cushion_ring), material=cushion_material, name="cushion_ring")

    for sign, name in ((1.0, "front_hinge_boss"), (-1.0, "rear_hinge_boss")):
        part.visual(
            Cylinder(radius=0.007, length=0.004),
            origin=Origin(
                xyz=(0.002, sign * 0.0265, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=trim_material,
            name=name,
        )

    if with_slider_slot:
        slot_surround = ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.014, 0.036, 0.005, corner_segments=8),
            [rounded_rect_profile(0.007, 0.024, 0.0035, corner_segments=8)],
            height=0.0014,
            center=True,
        ).rotate_y(math.pi / 2.0).translate(0.00125, -0.012, -0.001)
        part.visual(
            _mesh(f"{shell_mesh_name}_slot_surround", slot_surround),
            material=trim_material,
            name="slot_surround",
        )
        part.visual(
            Box((0.0025, 0.010, 0.030)),
            origin=Origin(xyz=(0.00125, -0.012, -0.001)),
            material=trim_material,
            name="guide_slot",
        )
        part.visual(
            Box((0.020, 0.0025, 0.010)),
            origin=Origin(xyz=(0.008, -0.012, 0.014)),
            material=trim_material,
            name="slot_marker",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="noise_canceling_headphones")

    shell_black = model.material("shell_black", rgba=(0.12, 0.12, 0.13, 1.0))
    cushion_black = model.material("cushion_black", rgba=(0.18, 0.18, 0.19, 1.0))
    graphite = model.material("graphite", rgba=(0.25, 0.27, 0.30, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.55, 0.57, 0.60, 1.0))

    headband = model.part("headband")
    band_path = [
        (-0.084, 0.0, 0.024),
        (-0.072, 0.0, 0.070),
        (-0.040, 0.0, 0.104),
        (0.000, 0.0, 0.116),
        (0.040, 0.0, 0.104),
        (0.072, 0.0, 0.070),
        (0.084, 0.0, 0.024),
    ]
    headband_outer = sweep_profile_along_spline(
        band_path,
        profile=rounded_rect_profile(0.034, 0.012, 0.0045, corner_segments=8),
        samples_per_segment=16,
        up_hint=(0.0, 1.0, 0.0),
    )
    headband_pad = sweep_profile_along_spline(
        [
            (-0.066, 0.0, 0.044),
            (-0.040, 0.0, 0.078),
            (0.000, 0.0, 0.090),
            (0.040, 0.0, 0.078),
            (0.066, 0.0, 0.044),
        ],
        profile=rounded_rect_profile(0.024, 0.008, 0.003, corner_segments=8),
        samples_per_segment=16,
        up_hint=(0.0, 1.0, 0.0),
    )
    headband.visual(_mesh("headband_outer", headband_outer), material=shell_black, name="band_outer")
    headband.visual(_mesh("headband_pad", headband_pad), material=cushion_black, name="band_pad")
    headband.visual(
        Box((0.016, 0.020, 0.012)),
        origin=Origin(xyz=(-0.084, 0.0, 0.026)),
        material=graphite,
        name="left_mount_block",
    )
    headband.visual(
        Box((0.016, 0.020, 0.012)),
        origin=Origin(xyz=(0.084, 0.0, 0.026)),
        material=graphite,
        name="right_mount_block",
    )
    headband.inertial = Inertial.from_geometry(
        Box((0.190, 0.040, 0.130)),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
    )

    left_yoke = model.part("left_yoke")
    _add_yoke_geometry(
        left_yoke,
        mesh_name="left_yoke_frame",
        frame_material=graphite,
        metal_material=satin_metal,
    )
    left_yoke.inertial = Inertial.from_geometry(
        Box((0.020, 0.070, 0.078)),
        mass=0.06,
        origin=Origin(xyz=(-0.002, 0.0, -0.036)),
    )

    right_yoke = model.part("right_yoke")
    _add_yoke_geometry(
        right_yoke,
        mesh_name="right_yoke_frame",
        frame_material=graphite,
        metal_material=satin_metal,
    )
    right_yoke.inertial = Inertial.from_geometry(
        Box((0.020, 0.070, 0.078)),
        mass=0.06,
        origin=Origin(xyz=(-0.002, 0.0, -0.036)),
    )

    left_earcup = model.part("left_earcup")
    _add_earcup_geometry(
        left_earcup,
        shell_mesh_name="left_earcup_shell",
        frame_mesh_name="left_earcup_frame",
        cushion_mesh_name="left_earcup_cushion",
        shell_material=shell_black,
        cushion_material=cushion_black,
        trim_material=graphite,
        with_slider_slot=False,
    )
    left_earcup.inertial = Inertial.from_geometry(
        Box((0.054, 0.090, 0.108)),
        mass=0.17,
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
    )

    right_earcup = model.part("right_earcup")
    _add_earcup_geometry(
        right_earcup,
        shell_mesh_name="right_earcup_shell",
        frame_mesh_name="right_earcup_frame",
        cushion_mesh_name="right_earcup_cushion",
        shell_material=shell_black,
        cushion_material=cushion_black,
        trim_material=graphite,
        with_slider_slot=True,
    )
    right_earcup.inertial = Inertial.from_geometry(
        Box((0.054, 0.090, 0.108)),
        mass=0.17,
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
    )

    control_slider = model.part("control_slider")
    control_slider.visual(
        Box((0.006, 0.010, 0.012)),
        origin=Origin(xyz=(-0.0034, 0.0, 0.006)),
        material=satin_metal,
        name="switch_cap",
    )
    control_slider.visual(
        Box((0.002, 0.012, 0.004)),
        origin=Origin(xyz=(-0.0054, 0.0, 0.010)),
        material=satin_metal,
        name="switch_rib",
    )
    control_slider.inertial = Inertial.from_geometry(
        Box((0.008, 0.014, 0.014)),
        mass=0.008,
        origin=Origin(xyz=(-0.0030, 0.0, 0.007)),
    )

    model.articulation(
        "headband_to_left_yoke",
        ArticulationType.FIXED,
        parent=headband,
        child=left_yoke,
        origin=Origin(xyz=(-0.084, 0.0, 0.020)),
    )
    model.articulation(
        "headband_to_right_yoke",
        ArticulationType.FIXED,
        parent=headband,
        child=right_yoke,
        origin=Origin(xyz=(0.084, 0.0, 0.020), rpy=(0.0, 0.0, math.pi)),
    )
    model.articulation(
        "left_fold",
        ArticulationType.REVOLUTE,
        parent=left_yoke,
        child=left_earcup,
        origin=Origin(xyz=(0.0, 0.0, -0.062)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "right_fold",
        ArticulationType.REVOLUTE,
        parent=right_yoke,
        child=right_earcup,
        origin=Origin(xyz=(0.0, 0.0, -0.062)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "right_cup_to_slider",
        ArticulationType.PRISMATIC,
        parent=right_earcup,
        child=control_slider,
        origin=Origin(xyz=(0.0004, -0.012, -0.015)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=0.05,
            lower=0.0,
            upper=0.012,
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
    headband = object_model.get_part("headband")
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_earcup = object_model.get_part("left_earcup")
    right_earcup = object_model.get_part("right_earcup")
    control_slider = object_model.get_part("control_slider")

    left_fold = object_model.get_articulation("left_fold")
    right_fold = object_model.get_articulation("right_fold")
    slider_joint = object_model.get_articulation("right_cup_to_slider")

    ctx.check(
        "all authored parts are present",
        all(
            part is not None
            for part in (headband, left_yoke, right_yoke, left_earcup, right_earcup, control_slider)
        ),
    )

    ctx.expect_contact(
        headband,
        left_yoke,
        elem_a="left_mount_block",
        elem_b="top_mount",
        contact_tol=0.0005,
        name="left yoke is mounted to the headband",
    )
    ctx.expect_contact(
        headband,
        right_yoke,
        elem_a="right_mount_block",
        elem_b="top_mount",
        contact_tol=0.0005,
        name="right yoke is mounted to the headband",
    )
    ctx.expect_contact(
        left_yoke,
        left_earcup,
        elem_a="front_pivot_pad",
        elem_b="front_hinge_boss",
        contact_tol=0.0005,
        name="left front hinge pad seats on the earcup boss",
    )
    ctx.expect_contact(
        left_yoke,
        left_earcup,
        elem_a="rear_pivot_pad",
        elem_b="rear_hinge_boss",
        contact_tol=0.0005,
        name="left rear hinge pad seats on the earcup boss",
    )
    ctx.expect_contact(
        right_yoke,
        right_earcup,
        elem_a="front_pivot_pad",
        elem_b="front_hinge_boss",
        contact_tol=0.0005,
        name="right front hinge pad seats on the earcup boss",
    )
    ctx.expect_contact(
        right_yoke,
        right_earcup,
        elem_a="rear_pivot_pad",
        elem_b="rear_hinge_boss",
        contact_tol=0.0005,
        name="right rear hinge pad seats on the earcup boss",
    )
    ctx.expect_origin_gap(
        headband,
        left_earcup,
        axis="z",
        min_gap=0.035,
        name="left earcup hangs below the headband",
    )
    ctx.expect_origin_gap(
        headband,
        right_earcup,
        axis="z",
        min_gap=0.035,
        name="right earcup hangs below the headband",
    )

    left_rest_aabb = ctx.part_element_world_aabb(left_earcup, elem="shell_cap")
    right_rest_aabb = ctx.part_element_world_aabb(right_earcup, elem="shell_cap")
    with ctx.pose({left_fold: left_fold.motion_limits.upper}):
        left_folded_aabb = ctx.part_element_world_aabb(left_earcup, elem="shell_cap")
    with ctx.pose({right_fold: right_fold.motion_limits.upper}):
        right_folded_aabb = ctx.part_element_world_aabb(right_earcup, elem="shell_cap")

    def _aabb_center_z(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return 0.5 * (lower[2] + upper[2])

    ctx.check(
        "left earcup folds upward",
        _aabb_center_z(left_rest_aabb) is not None
        and _aabb_center_z(left_folded_aabb) is not None
        and _aabb_center_z(left_folded_aabb) > _aabb_center_z(left_rest_aabb) + 0.008,
        details=f"rest_aabb={left_rest_aabb}, folded_aabb={left_folded_aabb}",
    )
    ctx.check(
        "right earcup folds upward",
        _aabb_center_z(right_rest_aabb) is not None
        and _aabb_center_z(right_folded_aabb) is not None
        and _aabb_center_z(right_folded_aabb) > _aabb_center_z(right_rest_aabb) + 0.008,
        details=f"rest_aabb={right_rest_aabb}, folded_aabb={right_folded_aabb}",
    )

    with ctx.pose({slider_joint: 0.0}):
        ctx.expect_contact(
            control_slider,
            right_earcup,
            elem_a="switch_cap",
            elem_b="guide_slot",
            contact_tol=0.0002,
            name="slider cap touches the guide wall at rest",
        )
        ctx.expect_within(
            control_slider,
            right_earcup,
            axes="yz",
            inner_elem="switch_cap",
            outer_elem="guide_slot",
            margin=0.001,
            name="slider cap stays within the guide slot at rest",
        )
        slider_rest = ctx.part_world_position(control_slider)

    with ctx.pose({slider_joint: slider_joint.motion_limits.upper}):
        ctx.expect_within(
            control_slider,
            right_earcup,
            axes="yz",
            inner_elem="switch_cap",
            outer_elem="guide_slot",
            margin=0.001,
            name="slider cap stays within the guide slot when extended",
        )
        slider_extended = ctx.part_world_position(control_slider)

    ctx.check(
        "slider switch travels upward along its guide",
        slider_rest is not None and slider_extended is not None and slider_extended[2] > slider_rest[2] + 0.008,
        details=f"rest={slider_rest}, extended={slider_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
