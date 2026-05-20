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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _lens_profile(width: float, height: float) -> list[tuple[float, float]]:
    return rounded_rect_profile(
        width,
        height,
        radius=min(width, height) * 0.18,
        corner_segments=10,
    )


def _rim_mesh(name: str, *, width: float, height: float, rim: float, thickness: float):
    geom = ExtrudeWithHolesGeometry(
        _lens_profile(width + 2.0 * rim, height + 2.0 * rim),
        [_lens_profile(width, height)],
        thickness,
        center=True,
    ).rotate_x(pi / 2.0)
    return _mesh(name, geom)


def _lens_mesh(name: str, *, width: float, height: float, thickness: float):
    geom = ExtrudeGeometry(
        _lens_profile(width, height),
        thickness,
        center=True,
    ).rotate_x(pi / 2.0)
    return _mesh(name, geom)


def _bridge_mesh(name: str, *, span: float, rise: float, radius: float):
    geom = tube_from_spline_points(
        [
            (-span * 0.5, 0.0, 0.0),
            (0.0, 0.0, rise),
            (span * 0.5, 0.0, 0.0),
        ],
        radius=radius,
        samples_per_segment=18,
        radial_segments=14,
    )
    return _mesh(name, geom)


def _temple_mesh(
    name: str,
    points: list[tuple[float, float, float]],
    *,
    width: float,
    thickness: float,
):
    profile = rounded_rect_profile(
        width,
        thickness,
        radius=min(width, thickness) * 0.32,
        corner_segments=6,
    )
    geom = sweep_profile_along_spline(
        points,
        profile=profile,
        samples_per_segment=16,
        cap_profile=True,
    )
    return _mesh(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flip_up_driving_glasses")

    gunmetal = model.material("gunmetal", rgba=(0.22, 0.23, 0.25, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.60, 0.62, 0.66, 1.0))
    temple_black = model.material("temple_black", rgba=(0.10, 0.10, 0.11, 1.0))
    clear_lens = model.material("clear_lens", rgba=(0.84, 0.90, 0.97, 0.30))
    driving_tint = model.material("driving_tint", rgba=(0.58, 0.42, 0.14, 0.46))

    lens_width = 0.053
    lens_height = 0.037
    lens_center_x = 0.036
    rim_width = 0.0034
    frame_depth = 0.0036

    primary_rim_mesh = _rim_mesh(
        "primary_rim",
        width=lens_width,
        height=lens_height,
        rim=rim_width,
        thickness=frame_depth,
    )
    primary_lens_mesh = _lens_mesh(
        "primary_lens",
        width=lens_width + 0.0008,
        height=lens_height + 0.0008,
        thickness=0.0012,
    )
    bridge_mesh = _bridge_mesh(
        "main_bridge",
        span=0.020,
        rise=0.0045,
        radius=0.0016,
    )

    main_frame = model.part("main_frame")
    main_frame.visual(
        primary_rim_mesh,
        origin=Origin(xyz=(-lens_center_x, 0.0, 0.0)),
        material=gunmetal,
        name="left_rim",
    )
    main_frame.visual(
        primary_rim_mesh,
        origin=Origin(xyz=(lens_center_x, 0.0, 0.0)),
        material=gunmetal,
        name="right_rim",
    )
    main_frame.visual(
        primary_lens_mesh,
        origin=Origin(xyz=(-lens_center_x, 0.0, 0.0)),
        material=clear_lens,
        name="left_primary_lens",
    )
    main_frame.visual(
        primary_lens_mesh,
        origin=Origin(xyz=(lens_center_x, 0.0, 0.0)),
        material=clear_lens,
        name="right_primary_lens",
    )
    main_frame.visual(
        bridge_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=gunmetal,
        name="bridge",
    )
    main_frame.visual(
        Box((0.108, 0.0028, 0.0036)),
        origin=Origin(xyz=(0.0, 0.0, 0.0205)),
        material=gunmetal,
        name="brow_bar",
    )
    main_frame.visual(
        Box((0.007, 0.005, 0.011)),
        origin=Origin(xyz=(-0.067, 0.0005, 0.005)),
        material=hinge_steel,
        name="left_hinge_block",
    )
    main_frame.visual(
        Box((0.007, 0.005, 0.011)),
        origin=Origin(xyz=(0.067, 0.0005, 0.005)),
        material=hinge_steel,
        name="right_hinge_block",
    )
    main_frame.visual(
        Box((0.010, 0.004, 0.006)),
        origin=Origin(xyz=(-0.034, 0.001, 0.020)),
        material=hinge_steel,
        name="left_flip_bracket",
    )
    main_frame.visual(
        Box((0.010, 0.004, 0.006)),
        origin=Origin(xyz=(0.034, 0.001, 0.020)),
        material=hinge_steel,
        name="right_flip_bracket",
    )
    main_frame.visual(
        Cylinder(radius=0.0016, length=0.006),
        origin=Origin(xyz=(-0.034, 0.001, 0.023), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_steel,
        name="left_flip_pin_stub",
    )
    main_frame.visual(
        Cylinder(radius=0.0016, length=0.006),
        origin=Origin(xyz=(0.034, 0.001, 0.023), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_steel,
        name="right_flip_pin_stub",
    )
    main_frame.inertial = Inertial.from_geometry(
        Box((0.148, 0.020, 0.050)),
        mass=0.036,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )

    left_temple = model.part("left_temple")
    left_temple.visual(
        Box((0.008, 0.005, 0.009)),
        origin=Origin(xyz=(0.0, -0.0045, 0.0)),
        material=hinge_steel,
        name="left_temple_hinge_block",
    )
    left_temple.visual(
        _temple_mesh(
            "left_temple_arm",
            [
                (0.0, -0.004, 0.0),
                (-0.010, -0.038, -0.001),
                (-0.009, -0.078, -0.006),
                (-0.004, -0.110, -0.012),
            ],
            width=0.0052,
            thickness=0.0025,
        ),
        material=gunmetal,
        name="left_temple_arm",
    )
    left_temple.visual(
        _temple_mesh(
            "left_temple_tip",
            [
                (-0.008, -0.075, -0.005),
                (-0.004, -0.112, -0.012),
                (0.0, -0.145, -0.022),
            ],
            width=0.0072,
            thickness=0.0034,
        ),
        material=temple_black,
        name="left_temple_tip",
    )
    left_temple.inertial = Inertial.from_geometry(
        Box((0.020, 0.150, 0.030)),
        mass=0.010,
        origin=Origin(xyz=(-0.004, -0.075, -0.010)),
    )

    right_temple = model.part("right_temple")
    right_temple.visual(
        Box((0.008, 0.005, 0.009)),
        origin=Origin(xyz=(0.0, -0.0045, 0.0)),
        material=hinge_steel,
        name="right_temple_hinge_block",
    )
    right_temple.visual(
        _temple_mesh(
            "right_temple_arm",
            [
                (0.0, -0.004, 0.0),
                (0.010, -0.038, -0.001),
                (0.009, -0.078, -0.006),
                (0.004, -0.110, -0.012),
            ],
            width=0.0052,
            thickness=0.0025,
        ),
        material=gunmetal,
        name="right_temple_arm",
    )
    right_temple.visual(
        _temple_mesh(
            "right_temple_tip",
            [
                (0.008, -0.075, -0.005),
                (0.004, -0.112, -0.012),
                (0.0, -0.145, -0.022),
            ],
            width=0.0072,
            thickness=0.0034,
        ),
        material=temple_black,
        name="right_temple_tip",
    )
    right_temple.inertial = Inertial.from_geometry(
        Box((0.020, 0.150, 0.030)),
        mass=0.010,
        origin=Origin(xyz=(0.004, -0.075, -0.010)),
    )

    tint_rim_mesh = _rim_mesh(
        "tint_rim",
        width=lens_width + 0.003,
        height=lens_height + 0.002,
        rim=0.0030,
        thickness=0.0030,
    )
    tint_lens_mesh = _lens_mesh(
        "tint_lens",
        width=lens_width + 0.0038,
        height=lens_height + 0.0028,
        thickness=0.0012,
    )
    tint_bridge_mesh = _bridge_mesh(
        "tint_bridge",
        span=0.024,
        rise=0.0048,
        radius=0.0015,
    )

    tinted_front = model.part("tinted_front")
    tinted_front.visual(
        Box((0.008, 0.004, 0.006)),
        origin=Origin(xyz=(-0.034, 0.002, -0.003)),
        material=hinge_steel,
        name="left_flip_tab",
    )
    tinted_front.visual(
        Box((0.008, 0.004, 0.006)),
        origin=Origin(xyz=(0.034, 0.002, -0.003)),
        material=hinge_steel,
        name="right_flip_tab",
    )
    tinted_front.visual(
        Cylinder(radius=0.0018, length=0.006),
        origin=Origin(xyz=(-0.034, 0.0015, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_steel,
        name="left_flip_barrel",
    )
    tinted_front.visual(
        Cylinder(radius=0.0018, length=0.006),
        origin=Origin(xyz=(0.034, 0.0015, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_steel,
        name="right_flip_barrel",
    )
    tinted_front.visual(
        Box((0.102, 0.0040, 0.0034)),
        origin=Origin(xyz=(0.0, 0.0045, -0.005)),
        material=gunmetal,
        name="tint_brow_bar",
    )
    tinted_front.visual(
        tint_rim_mesh,
        origin=Origin(xyz=(-lens_center_x, 0.008, -0.023)),
        material=gunmetal,
        name="left_tint_rim",
    )
    tinted_front.visual(
        tint_rim_mesh,
        origin=Origin(xyz=(lens_center_x, 0.008, -0.023)),
        material=gunmetal,
        name="right_tint_rim",
    )
    tinted_front.visual(
        tint_lens_mesh,
        origin=Origin(xyz=(-lens_center_x, 0.008, -0.023)),
        material=driving_tint,
        name="left_tint_lens",
    )
    tinted_front.visual(
        tint_lens_mesh,
        origin=Origin(xyz=(lens_center_x, 0.008, -0.023)),
        material=driving_tint,
        name="right_tint_lens",
    )
    tinted_front.visual(
        tint_bridge_mesh,
        origin=Origin(xyz=(0.0, 0.008, -0.026)),
        material=gunmetal,
        name="tint_bridge",
    )
    tinted_front.inertial = Inertial.from_geometry(
        Box((0.150, 0.020, 0.058)),
        mass=0.016,
        origin=Origin(xyz=(0.0, 0.008, -0.020)),
    )

    model.articulation(
        "left_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=left_temple,
        origin=Origin(xyz=(-0.067, 0.0, 0.005)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=3.0,
            lower=0.0,
            upper=1.28,
        ),
    )
    model.articulation(
        "right_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=right_temple,
        origin=Origin(xyz=(0.067, 0.0, 0.005)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=3.0,
            lower=0.0,
            upper=1.28,
        ),
    )
    model.articulation(
        "flip_front_pivot",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=tinted_front,
        origin=Origin(xyz=(0.0, 0.003, 0.023)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=2.0,
            lower=0.0,
            upper=1.95,
        ),
    )

    return model


def _aabb_center(aabb):
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    main_frame = object_model.get_part("main_frame")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")
    tinted_front = object_model.get_part("tinted_front")

    left_temple_hinge = object_model.get_articulation("left_temple_hinge")
    right_temple_hinge = object_model.get_articulation("right_temple_hinge")
    flip_front_pivot = object_model.get_articulation("flip_front_pivot")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "temple hinge axes are vertical",
        left_temple_hinge.axis == (0.0, 0.0, 1.0) and right_temple_hinge.axis == (0.0, 0.0, -1.0),
        details=f"left={left_temple_hinge.axis}, right={right_temple_hinge.axis}",
    )
    ctx.check(
        "flip front axis is transverse",
        flip_front_pivot.axis == (1.0, 0.0, 0.0),
        details=f"axis={flip_front_pivot.axis}",
    )

    ctx.expect_contact(
        left_temple,
        main_frame,
        elem_a="left_temple_hinge_block",
        elem_b="left_hinge_block",
        name="left temple is mounted to outer hinge block",
    )
    ctx.expect_contact(
        right_temple,
        main_frame,
        elem_a="right_temple_hinge_block",
        elem_b="right_hinge_block",
        name="right temple is mounted to outer hinge block",
    )
    ctx.expect_contact(
        tinted_front,
        main_frame,
        elem_a="left_flip_tab",
        elem_b="left_flip_bracket",
        name="left flip hinge tab seats on brow bracket",
    )
    ctx.expect_contact(
        tinted_front,
        main_frame,
        elem_a="right_flip_tab",
        elem_b="right_flip_bracket",
        name="right flip hinge tab seats on brow bracket",
    )

    ctx.expect_gap(
        tinted_front,
        main_frame,
        axis="y",
        positive_elem="left_tint_lens",
        negative_elem="left_primary_lens",
        min_gap=0.005,
        max_gap=0.014,
        name="tinted left lens sits ahead of clear left lens",
    )
    ctx.expect_gap(
        tinted_front,
        main_frame,
        axis="y",
        positive_elem="right_tint_lens",
        negative_elem="right_primary_lens",
        min_gap=0.005,
        max_gap=0.014,
        name="tinted right lens sits ahead of clear right lens",
    )
    ctx.expect_overlap(
        tinted_front,
        main_frame,
        axes="xz",
        elem_a="left_tint_lens",
        elem_b="left_primary_lens",
        min_overlap=0.024,
        name="left tint lens covers primary left opening",
    )
    ctx.expect_overlap(
        tinted_front,
        main_frame,
        axes="xz",
        elem_a="right_tint_lens",
        elem_b="right_primary_lens",
        min_overlap=0.024,
        name="right tint lens covers primary right opening",
    )

    left_tip_open_aabb = ctx.part_element_world_aabb(left_temple, elem="left_temple_tip")
    right_tip_open_aabb = ctx.part_element_world_aabb(right_temple, elem="right_temple_tip")
    assert left_tip_open_aabb is not None
    assert right_tip_open_aabb is not None
    left_tip_open = _aabb_center(left_tip_open_aabb)
    right_tip_open = _aabb_center(right_tip_open_aabb)

    with ctx.pose({left_temple_hinge: 1.10, right_temple_hinge: 1.10}):
        left_tip_folded_aabb = ctx.part_element_world_aabb(left_temple, elem="left_temple_tip")
        right_tip_folded_aabb = ctx.part_element_world_aabb(right_temple, elem="right_temple_tip")
        assert left_tip_folded_aabb is not None
        assert right_tip_folded_aabb is not None
        left_tip_folded = _aabb_center(left_tip_folded_aabb)
        right_tip_folded = _aabb_center(right_tip_folded_aabb)

        ctx.check(
            "left temple folds inward",
            left_tip_folded[0] > left_tip_open[0] + 0.060,
            details=f"open_x={left_tip_open[0]:.4f}, folded_x={left_tip_folded[0]:.4f}",
        )
        ctx.check(
            "right temple folds inward",
            right_tip_folded[0] < right_tip_open[0] - 0.060,
            details=f"open_x={right_tip_open[0]:.4f}, folded_x={right_tip_folded[0]:.4f}",
        )

    with ctx.pose({flip_front_pivot: 1.65}):
        ctx.expect_gap(
            tinted_front,
            main_frame,
            axis="z",
            positive_elem="left_tint_lens",
            negative_elem="left_primary_lens",
            min_gap=0.010,
            name="flipped front clears above left primary lens",
        )
        ctx.expect_gap(
            tinted_front,
            main_frame,
            axis="z",
            positive_elem="right_tint_lens",
            negative_elem="right_primary_lens",
            min_gap=0.010,
            name="flipped front clears above right primary lens",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
