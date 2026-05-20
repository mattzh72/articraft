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
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_tackle_box")

    body_polymer = model.material("body_polymer", rgba=(0.28, 0.33, 0.27, 1.0))
    hardware_metal = model.material("hardware_metal", rgba=(0.72, 0.75, 0.78, 1.0))
    latch_polymer = model.material("latch_polymer", rgba=(0.12, 0.14, 0.15, 1.0))
    gasket_rubber = model.material("gasket_rubber", rgba=(0.08, 0.09, 0.10, 1.0))

    base_length = 0.46
    base_width = 0.28
    base_height = 0.165
    wall_thickness = 0.006
    bottom_thickness = 0.008
    lid_length = 0.476
    lid_width = 0.296
    lid_inner_length = 0.464
    lid_inner_width = 0.284
    lid_skirt_bottom = -0.045
    lid_skirt_top = 0.013
    lid_top_thickness = 0.008
    hinge_axis_z = 0.152
    hinge_axis_x = -(base_length / 2.0) - 0.016
    lid_shell_x = 0.246
    gasket_x = 0.246

    def _ring_mesh(
        *,
        outer_x: float,
        outer_y: float,
        inner_x: float,
        inner_y: float,
        height: float,
        outer_r: float,
        inner_r: float,
        z0: float,
    ):
        geom = ExtrudeWithHolesGeometry(
            rounded_rect_profile(outer_x, outer_y, outer_r),
            [rounded_rect_profile(inner_x, inner_y, inner_r)],
            height,
            center=True,
        )
        geom.translate(0.0, 0.0, z0 + (height / 2.0))
        return geom

    def _panel_mesh(*, x: float, y: float, r: float, z0: float, thickness: float):
        geom = ExtrudeGeometry.from_z0(rounded_rect_profile(x, y, r), thickness)
        geom.translate(0.0, 0.0, z0)
        return geom

    base_shell_geom = _panel_mesh(
        x=base_length,
        y=base_width,
        r=0.026,
        z0=0.0,
        thickness=bottom_thickness,
    )
    base_shell_geom.merge(
        _ring_mesh(
            outer_x=base_length,
            outer_y=base_width,
            inner_x=base_length - (2.0 * wall_thickness),
            inner_y=base_width - (2.0 * wall_thickness),
            height=base_height - bottom_thickness,
            outer_r=0.026,
            inner_r=0.020,
            z0=bottom_thickness,
        )
    )

    seal_land_geom = _ring_mesh(
        outer_x=base_length - (2.0 * wall_thickness),
        outer_y=base_width - (2.0 * wall_thickness),
        inner_x=0.412,
        inner_y=0.232,
        height=0.004,
        outer_r=0.020,
        inner_r=0.014,
        z0=0.157,
    )

    lid_shell_geom = _ring_mesh(
        outer_x=lid_length,
        outer_y=lid_width,
        inner_x=lid_inner_length,
        inner_y=lid_inner_width,
        height=lid_skirt_top - lid_skirt_bottom,
        outer_r=0.034,
        inner_r=0.028,
        z0=lid_skirt_bottom,
    )
    lid_shell_geom.merge(
        _panel_mesh(
            x=lid_length,
            y=lid_width,
            r=0.034,
            z0=lid_skirt_top,
            thickness=lid_top_thickness,
        )
    )
    lid_shell_geom.merge(
        _ring_mesh(
            outer_x=0.480,
            outer_y=0.300,
            inner_x=lid_inner_length,
            inner_y=lid_inner_width,
            height=0.004,
            outer_r=0.036,
            inner_r=0.028,
            z0=lid_skirt_bottom - 0.004,
        )
    )

    gasket_geom = _ring_mesh(
        outer_x=0.444,
        outer_y=0.264,
        inner_x=0.408,
        inner_y=0.228,
        height=0.004,
        outer_r=0.018,
        inner_r=0.012,
        z0=0.009,
    )

    base = model.part("base_shell")
    base.visual(mesh_from_geometry(base_shell_geom, "base_shell"), material=body_polymer, name="base_body")
    base.visual(mesh_from_geometry(seal_land_geom, "seal_land"), material=body_polymer, name="seal_land")
    for y_pos, side_name in ((0.092, "left"), (-0.092, "right")):
        base.visual(
            Box((0.012, 0.060, 0.022)),
            origin=Origin(xyz=(-0.235, y_pos, 0.091)),
            material=hardware_metal,
            name=f"{side_name}_hinge_leaf",
        )
        base.visual(
            Box((0.012, 0.060, 0.048)),
            origin=Origin(xyz=(-0.247, y_pos, 0.126)),
            material=hardware_metal,
            name=f"{side_name}_hinge_upright",
        )
        base.visual(
            Cylinder(radius=0.0055, length=0.060),
            origin=Origin(
                xyz=(hinge_axis_x, y_pos, hinge_axis_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hardware_metal,
            name=f"{side_name}_hinge_barrel",
        )
    for y_pos, side_name in ((0.090, "left"), (-0.090, "right")):
        base.visual(
            Box((0.017, 0.052, 0.020)),
            origin=Origin(xyz=((base_length / 2.0) + 0.0070, y_pos, 0.089)),
            material=hardware_metal,
            name=f"{side_name}_catch_body",
        )
        base.visual(
            Box((0.012, 0.052, 0.006)),
            origin=Origin(xyz=((base_length / 2.0) + 0.0095, y_pos, 0.099)),
            material=hardware_metal,
            name=f"{side_name}_catch_lip",
        )
    for x_pos, y_pos, foot_name in (
        (-0.150, 0.090, "foot_back_left"),
        (-0.150, -0.090, "foot_back_right"),
        (0.150, 0.090, "foot_front_left"),
        (0.150, -0.090, "foot_front_right"),
    ):
        base.visual(
            Box((0.060, 0.034, 0.008)),
            origin=Origin(xyz=(x_pos, y_pos, 0.004)),
            material=latch_polymer,
            name=foot_name,
        )
    base.inertial = Inertial.from_geometry(
        Box((base_length, base_width, base_height)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, base_height / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(lid_shell_geom, "lid_shell"),
        origin=Origin(xyz=(lid_shell_x, 0.0, 0.0)),
        material=body_polymer,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_geometry(gasket_geom, "lid_gasket"),
        origin=Origin(xyz=(gasket_x, 0.0, 0.0)),
        material=gasket_rubber,
        name="lid_gasket",
    )
    lid.visual(
        Box((0.028, 0.220, 0.016)),
        origin=Origin(xyz=(0.000, 0.0, 0.015)),
        material=body_polymer,
        name="rear_rain_hood",
    )
    lid.visual(
        Box((0.016, 0.240, 0.010)),
        origin=Origin(xyz=(lid_length + 0.009, 0.0, -0.042)),
        material=body_polymer,
        name="front_drip_rail",
    )
    lid.visual(
        Box((0.014, 0.108, 0.030)),
        origin=Origin(xyz=(-0.004, 0.0, 0.001)),
        material=hardware_metal,
        name="center_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_metal,
        name="center_hinge_barrel",
    )
    lid.visual(
        Box((0.014, 0.236, 0.018)),
        origin=Origin(xyz=(0.490, 0.0, 0.020)),
        material=hardware_metal,
        name="front_latch_housing",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_length, lid_width, 0.074)),
        mass=1.7,
        origin=Origin(xyz=(lid_length / 2.0, 0.0, -0.012)),
    )

    left_latch = model.part("left_latch")
    left_latch.visual(
        Cylinder(radius=0.005, length=0.046),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_metal,
        name="pivot_barrel",
    )
    left_latch.visual(
        Box((0.028, 0.046, 0.024)),
        origin=Origin(xyz=(0.008, 0.0, -0.010)),
        material=latch_polymer,
        name="latch_body",
    )
    left_latch.visual(
        Box((0.010, 0.030, 0.044)),
        origin=Origin(xyz=(0.018, 0.0, -0.042)),
        material=hardware_metal,
        name="hook_web",
    )
    left_latch.visual(
        Box((0.020, 0.046, 0.008)),
        origin=Origin(xyz=(0.020, 0.0, -0.062)),
        material=hardware_metal,
        name="hook",
    )
    left_latch.visual(
        Box((0.012, 0.030, 0.012)),
        origin=Origin(xyz=(0.020, 0.0, -0.008)),
        material=latch_polymer,
        name="thumb_tab",
    )
    left_latch.inertial = Inertial.from_geometry(
        Box((0.036, 0.046, 0.078)),
        mass=0.09,
        origin=Origin(xyz=(0.018, 0.0, -0.033)),
    )

    right_latch = model.part("right_latch")
    right_latch.visual(
        Cylinder(radius=0.005, length=0.046),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_metal,
        name="pivot_barrel",
    )
    right_latch.visual(
        Box((0.028, 0.046, 0.024)),
        origin=Origin(xyz=(0.008, 0.0, -0.010)),
        material=latch_polymer,
        name="latch_body",
    )
    right_latch.visual(
        Box((0.010, 0.030, 0.044)),
        origin=Origin(xyz=(0.018, 0.0, -0.042)),
        material=hardware_metal,
        name="hook_web",
    )
    right_latch.visual(
        Box((0.020, 0.046, 0.008)),
        origin=Origin(xyz=(0.020, 0.0, -0.062)),
        material=hardware_metal,
        name="hook",
    )
    right_latch.visual(
        Box((0.012, 0.030, 0.012)),
        origin=Origin(xyz=(0.020, 0.0, -0.008)),
        material=latch_polymer,
        name="thumb_tab",
    )
    right_latch.inertial = Inertial.from_geometry(
        Box((0.036, 0.046, 0.078)),
        mass=0.09,
        origin=Origin(xyz=(0.018, 0.0, -0.033)),
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=1.95,
        ),
    )
    model.articulation(
        "lid_to_left_latch",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=left_latch,
        origin=Origin(xyz=(0.486, 0.090, 0.010)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "lid_to_right_latch",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=right_latch,
        origin=Origin(xyz=(0.486, -0.090, 0.010)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_shell")
    lid = object_model.get_part("lid")
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")
    lid_hinge = object_model.get_articulation("base_to_lid")
    left_latch_joint = object_model.get_articulation("lid_to_left_latch")
    right_latch_joint = object_model.get_articulation("lid_to_right_latch")

    seal_land = base.get_visual("seal_land")
    lid_gasket = lid.get_visual("lid_gasket")
    front_drip_rail = lid.get_visual("front_drip_rail")
    left_catch_lip = base.get_visual("left_catch_lip")
    right_catch_lip = base.get_visual("right_catch_lip")
    left_hook = left_latch.get_visual("hook")
    right_hook = right_latch.get_visual("hook")

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

    with ctx.pose({lid_hinge: 0.0, left_latch_joint: 0.0, right_latch_joint: 0.0}):
        ctx.expect_overlap(lid, base, axes="xy", min_overlap=0.20, name="lid_covers_base_footprint")
        ctx.expect_contact(
            lid,
            base,
            elem_a=lid_gasket,
            elem_b=seal_land,
            contact_tol=0.001,
            name="gasket_seats_on_seal_land",
        )
        ctx.expect_gap(
            left_latch,
            base,
            axis="x",
            positive_elem=left_hook,
            negative_elem=left_catch_lip,
            min_gap=0.0,
            max_gap=0.006,
            name="left_latch_aligned_to_catch",
        )
        ctx.expect_gap(
            right_latch,
            base,
            axis="x",
            positive_elem=right_hook,
            negative_elem=right_catch_lip,
            min_gap=0.0,
            max_gap=0.006,
            name="right_latch_aligned_to_catch",
        )
        ctx.expect_overlap(
            left_latch,
            base,
            axes="y",
            elem_a=left_hook,
            elem_b=left_catch_lip,
            min_overlap=0.035,
            name="left_hook_tracks_left_catch",
        )
        ctx.expect_overlap(
            right_latch,
            base,
            axes="y",
            elem_a=right_hook,
            elem_b=right_catch_lip,
            min_overlap=0.035,
            name="right_hook_tracks_right_catch",
        )

    with ctx.pose({left_latch_joint: 1.10, right_latch_joint: 1.10}):
        ctx.expect_gap(
            left_latch,
            base,
            axis="x",
            positive_elem=left_hook,
            negative_elem=left_catch_lip,
            min_gap=0.020,
            name="left_latch_swings_clear_of_catch",
        )
        ctx.expect_gap(
            right_latch,
            base,
            axis="x",
            positive_elem=right_hook,
            negative_elem=right_catch_lip,
            min_gap=0.020,
            name="right_latch_swings_clear_of_catch",
        )

    with ctx.pose({lid_hinge: 1.10, left_latch_joint: 1.05, right_latch_joint: 1.05}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem=front_drip_rail,
            negative_elem=seal_land,
            min_gap=0.075,
            name="lid_front_rises_when_opened",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
