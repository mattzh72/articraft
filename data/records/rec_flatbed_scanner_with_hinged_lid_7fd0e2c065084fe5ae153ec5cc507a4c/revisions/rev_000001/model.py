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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="engineering_scanner")

    chassis_dark = model.material("chassis_dark", rgba=(0.22, 0.24, 0.26, 1.0))
    panel_gray = model.material("panel_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    roller_black = model.material("roller_black", rgba=(0.10, 0.10, 0.11, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.42, 0.58, 0.70, 0.38))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    indicator_blue = model.material("indicator_blue", rgba=(0.16, 0.44, 0.88, 1.0))

    body_width = 1.36
    body_depth = 0.58
    body_height = 0.16
    lid_width = 1.34
    lid_depth = 0.488
    lid_thickness = 0.036
    housing_width = 1.28
    housing_depth = 0.082
    housing_height = 0.046

    lid_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(lid_width, lid_depth, radius=0.020, corner_segments=8),
            lid_thickness,
            center=True,
            cap=True,
        ),
        "scanner_lid_shell",
    )
    housing_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(housing_width, housing_depth, radius=0.016, corner_segments=8),
            housing_height,
            center=True,
            cap=True,
        ),
        "scanner_roller_housing_cover",
    )

    chassis = model.part("chassis")
    chassis.visual(
        Box((body_width, body_depth, 0.110)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=chassis_dark,
        name="lower_body",
    )
    chassis.visual(
        Box((0.050, body_depth, body_height - 0.110)),
        origin=Origin(xyz=(-0.655, 0.0, 0.135)),
        material=chassis_dark,
        name="left_upper_wall",
    )
    chassis.visual(
        Box((0.050, body_depth, body_height - 0.110)),
        origin=Origin(xyz=(0.655, 0.0, 0.135)),
        material=chassis_dark,
        name="right_upper_wall",
    )
    chassis.visual(
        Box((1.260, 0.060, body_height - 0.110)),
        origin=Origin(xyz=(0.0, -0.260, 0.135)),
        material=chassis_dark,
        name="front_upper_wall",
    )
    chassis.visual(
        Box((1.260, 0.050, body_height - 0.110)),
        origin=Origin(xyz=(0.0, 0.265, 0.135)),
        material=chassis_dark,
        name="rear_upper_wall",
    )
    chassis.visual(
        Box((1.210, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, -0.285, 0.094)),
        material=roller_black,
        name="paper_slot",
    )
    chassis.visual(
        Box((0.150, 0.006, 0.024)),
        origin=Origin(xyz=(0.520, -0.287, 0.132)),
        material=indicator_blue,
        name="status_strip",
    )
    chassis.visual(
        Box((1.200, 0.032, 0.006)),
        origin=Origin(xyz=(0.0, -0.221, 0.143)),
        material=trim_gray,
        name="front_glass_ledge",
    )
    chassis.visual(
        Box((1.200, 0.032, 0.006)),
        origin=Origin(xyz=(0.0, 0.221, 0.143)),
        material=trim_gray,
        name="rear_glass_ledge",
    )
    chassis.visual(
        Box((0.036, 0.430, 0.006)),
        origin=Origin(xyz=(-0.617, 0.0, 0.143)),
        material=trim_gray,
        name="left_glass_ledge",
    )
    chassis.visual(
        Box((0.036, 0.430, 0.006)),
        origin=Origin(xyz=(0.617, 0.0, 0.143)),
        material=trim_gray,
        name="right_glass_ledge",
    )
    for index, x_pos in enumerate((-0.545, 0.545)):
        for jindex, y_pos in enumerate((-0.210, 0.210)):
            chassis.visual(
                Box((0.080, 0.055, 0.012)),
                origin=Origin(xyz=(x_pos, y_pos, -0.006)),
                material=rubber,
                name=f"foot_{index}_{jindex}",
            )
    chassis.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height + 0.012)),
        mass=56.0,
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
    )

    platen_glass = model.part("platen_glass")
    platen_glass.visual(
        Box((1.200, 0.430, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.149)),
        material=glass_blue,
        name="glass_panel",
    )
    platen_glass.inertial = Inertial.from_geometry(
        Box((1.200, 0.430, 0.006)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.149)),
    )

    lid = model.part("lid")
    lid.visual(
        lid_mesh,
        origin=Origin(xyz=(0.0, -0.247, 0.008)),
        material=panel_gray,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.013, length=1.300),
        origin=Origin(xyz=(0.0, 0.000, 0.001), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.035, 0.450, 0.003)),
        origin=Origin(xyz=(-0.6525, -0.245, -0.0115)),
        material=trim_gray,
        name="left_lid_seal",
    )
    lid.visual(
        Box((0.035, 0.450, 0.003)),
        origin=Origin(xyz=(0.6525, -0.245, -0.0115)),
        material=trim_gray,
        name="right_lid_seal",
    )
    lid.visual(
        Box((1.080, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.482, -0.005)),
        material=trim_gray,
        name="lid_front_pull",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_depth, 0.050)),
        mass=13.0,
        origin=Origin(xyz=(0.0, -0.244, 0.002)),
    )

    roller_housing = model.part("roller_housing")
    roller_housing.visual(
        housing_mesh,
        origin=Origin(xyz=(0.0, 0.050, 0.040)),
        material=panel_gray,
        name="housing_cover",
    )
    roller_housing.visual(
        Box((1.240, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, 0.010, 0.015)),
        material=trim_gray,
        name="housing_hinge_leaf",
    )
    roller_housing.visual(
        Box((0.016, 0.080, 0.055)),
        origin=Origin(xyz=(-0.632, 0.050, 0.0275)),
        material=trim_gray,
        name="left_housing_cheek",
    )
    roller_housing.visual(
        Box((0.016, 0.080, 0.055)),
        origin=Origin(xyz=(0.632, 0.050, 0.0275)),
        material=trim_gray,
        name="right_housing_cheek",
    )
    roller_housing.visual(
        Cylinder(radius=0.005, length=1.250),
        origin=Origin(xyz=(0.0, 0.055, 0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="roller_axle",
    )
    roller_housing.visual(
        Cylinder(radius=0.017, length=1.220),
        origin=Origin(xyz=(0.0, 0.055, 0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=roller_black,
        name="feed_roller",
    )
    roller_housing.visual(
        Box((1.220, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, 0.083, 0.018)),
        material=trim_gray,
        name="rear_paper_guide",
    )
    roller_housing.inertial = Inertial.from_geometry(
        Box((housing_width, housing_depth, 0.070)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.048, 0.032)),
    )

    model.articulation(
        "chassis_to_platen_glass",
        ArticulationType.FIXED,
        parent=chassis,
        child=platen_glass,
        origin=Origin(),
    )
    model.articulation(
        "rear_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=lid,
        origin=Origin(xyz=(0.0, 0.290, 0.173)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.8,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "front_roller_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=roller_housing,
        origin=Origin(xyz=(0.0, -0.290, 0.160)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    platen_glass = object_model.get_part("platen_glass")
    lid = object_model.get_part("lid")
    roller_housing = object_model.get_part("roller_housing")
    rear_lid_hinge = object_model.get_articulation("rear_lid_hinge")
    front_roller_hinge = object_model.get_articulation("front_roller_hinge")

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

    ctx.check("has_chassis", chassis.name == "chassis", "Missing chassis part.")
    ctx.check("has_platen_glass", platen_glass.name == "platen_glass", "Missing platen glass part.")
    ctx.check("has_lid", lid.name == "lid", "Missing full-width lid part.")
    ctx.check("has_roller_housing", roller_housing.name == "roller_housing", "Missing roller housing part.")

    ctx.check(
        "rear_lid_hinge_axis",
        tuple(rear_lid_hinge.axis) == (-1.0, 0.0, 0.0),
        f"Rear lid hinge axis should run across scanner width, got {rear_lid_hinge.axis}.",
    )
    ctx.check(
        "front_roller_hinge_axis",
        tuple(front_roller_hinge.axis) == (1.0, 0.0, 0.0),
        f"Roller housing hinge axis should run across scanner width, got {front_roller_hinge.axis}.",
    )

    ctx.expect_contact(platen_glass, chassis, name="platen_supported_on_ledges")
    ctx.expect_contact(lid, chassis, name="lid_closed_on_chassis")
    ctx.expect_contact(roller_housing, chassis, name="roller_housing_closed_on_front_edge")
    ctx.expect_within(platen_glass, chassis, axes="xy", margin=0.0, name="platen_within_body_footprint")
    ctx.expect_gap(
        lid,
        platen_glass,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="glass_panel",
        min_gap=0.008,
        max_gap=0.018,
        name="lid_clears_glass_when_closed",
    )

    with ctx.pose({rear_lid_hinge: 1.10}):
        ctx.expect_gap(
            lid,
            chassis,
            axis="z",
            positive_elem="lid_front_pull",
            min_gap=0.20,
            name="lid_front_edge_lifts_clear_when_open",
        )

    with ctx.pose({front_roller_hinge: 1.00}):
        ctx.expect_gap(
            roller_housing,
            chassis,
            axis="z",
            positive_elem="rear_paper_guide",
            min_gap=0.05,
            name="roller_housing_rear_edge_flips_up_for_loading",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
