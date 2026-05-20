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
    rounded_rect_profile,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lockable_underground_pit_electrical_cabinet")

    powder_coat = model.material("powder_coat", rgba=(0.29, 0.31, 0.33, 1.0))
    lid_finish = model.material("lid_finish", rgba=(0.40, 0.42, 0.44, 1.0))
    galvanized = model.material("galvanized", rgba=(0.72, 0.74, 0.76, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.08, 0.08, 1.0))

    outer = 0.62
    depth = 0.24
    wall = 0.025
    frame_band = 0.035
    hinge_axis_x = -(outer * 0.5 + 0.008)
    hinge_axis_z = 0.014

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    housing = model.part("housing")
    housing.inertial = Inertial.from_geometry(
        Box((outer, outer, depth + 0.04)),
        mass=62.0,
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
    )

    housing.visual(
        Box((outer - 2.0 * wall, outer - 2.0 * wall, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -depth + 0.01)),
        material=powder_coat,
        name="sump_floor",
    )
    housing.visual(
        Box((wall, outer, depth)),
        origin=Origin(xyz=(outer * 0.5 - wall * 0.5, 0.0, -depth * 0.5)),
        material=powder_coat,
        name="front_wall",
    )
    housing.visual(
        Box((wall, outer, depth)),
        origin=Origin(xyz=(-(outer * 0.5 - wall * 0.5), 0.0, -depth * 0.5)),
        material=powder_coat,
        name="rear_wall",
    )
    housing.visual(
        Box((outer - 2.0 * wall, wall, depth)),
        origin=Origin(xyz=(0.0, outer * 0.5 - wall * 0.5, -depth * 0.5)),
        material=powder_coat,
        name="right_wall",
    )
    housing.visual(
        Box((outer - 2.0 * wall, wall, depth)),
        origin=Origin(xyz=(0.0, -(outer * 0.5 - wall * 0.5), -depth * 0.5)),
        material=powder_coat,
        name="left_wall",
    )

    housing.visual(
        Box((frame_band, outer, 0.012)),
        origin=Origin(xyz=(outer * 0.5 - frame_band * 0.5, 0.0, -0.006)),
        material=powder_coat,
        name="front_frame",
    )
    housing.visual(
        Box((frame_band, outer, 0.012)),
        origin=Origin(xyz=(-(outer * 0.5 - frame_band * 0.5), 0.0, -0.006)),
        material=powder_coat,
        name="rear_frame",
    )
    housing.visual(
        Box((outer - 2.0 * frame_band, frame_band, 0.012)),
        origin=Origin(xyz=(0.0, outer * 0.5 - frame_band * 0.5, -0.006)),
        material=powder_coat,
        name="right_frame",
    )
    housing.visual(
        Box((outer - 2.0 * frame_band, frame_band, 0.012)),
        origin=Origin(xyz=(0.0, -(outer * 0.5 - frame_band * 0.5), -0.006)),
        material=powder_coat,
        name="left_frame",
    )

    for side in (-1.0, 1.0):
        cluster_y = side * 0.20
        for y_center in (cluster_y - 0.032, cluster_y + 0.032):
            housing.visual(
                Box((0.028, 0.024, 0.028)),
                origin=Origin(xyz=(hinge_axis_x + 0.012, y_center, -0.009)),
                material=galvanized,
            )
            housing.visual(
                Cylinder(radius=0.0085, length=0.020),
                origin=Origin(xyz=(hinge_axis_x, y_center, hinge_axis_z - 0.008), rpy=(pi / 2.0, 0.0, 0.0)),
                material=galvanized,
            )

    lid = model.part("lid")
    lid_length = 0.61
    lid_width = 0.58
    lid_top_z = 0.004
    lid.inertial = Inertial.from_geometry(
        Box((lid_length, lid_width, 0.055)),
        mass=19.0,
        origin=Origin(xyz=(lid_length * 0.50, 0.0, 0.000)),
    )

    lid.visual(
        Box((0.46, lid_width, 0.020)),
        origin=Origin(xyz=(0.260, 0.0, lid_top_z)),
        material=lid_finish,
        name="lid_rear_field",
    )
    lid.visual(
        Box((0.09, 0.29, 0.020)),
        origin=Origin(xyz=(0.580, 0.0, lid_top_z)),
        material=lid_finish,
        name="lid_front_band",
    )
    lid.visual(
        Box((0.11, 0.115, 0.020)),
        origin=Origin(xyz=(0.550, 0.0875, lid_top_z)),
        material=lid_finish,
        name="lid_right_pocket_band",
    )
    lid.visual(
        Box((0.11, 0.115, 0.020)),
        origin=Origin(xyz=(0.550, -0.0875, lid_top_z)),
        material=lid_finish,
        name="lid_left_pocket_band",
    )
    lid.visual(
        Box((0.095, 0.145, 0.006)),
        origin=Origin(xyz=(0.548, 0.0, -0.004)),
        material=gasket_black,
        name="hasp_recess_floor",
    )
    lid.visual(
        Box((0.095, 0.012, 0.020)),
        origin=Origin(xyz=(0.548, 0.0725, 0.003)),
        material=lid_finish,
    )
    lid.visual(
        Box((0.095, 0.012, 0.020)),
        origin=Origin(xyz=(0.548, -0.0725, 0.003)),
        material=lid_finish,
    )
    lid.visual(
        Box((0.012, 0.145, 0.020)),
        origin=Origin(xyz=(0.4945, 0.0, 0.003)),
        material=lid_finish,
    )
    lid.visual(
        Box((0.48, 0.012, 0.030)),
        origin=Origin(xyz=(0.300, lid_width * 0.5 - 0.026, -0.014)),
        material=lid_finish,
        name="right_skirt",
    )
    lid.visual(
        Box((0.48, 0.012, 0.030)),
        origin=Origin(xyz=(0.300, -(lid_width * 0.5 - 0.026), -0.014)),
        material=lid_finish,
        name="left_skirt",
    )
    for side in (-1.0, 1.0):
        cluster_y = side * 0.20
        lid.visual(
            Box((0.030, 0.044, 0.024)),
            origin=Origin(xyz=(0.015, cluster_y, 0.000)),
            material=galvanized,
        )
        lid.visual(
            Cylinder(radius=0.008, length=0.042),
            origin=Origin(xyz=(0.0, cluster_y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"lid_hinge_barrel_{'right' if side > 0.0 else 'left'}",
        )

    for y_center in (-0.048, 0.048):
        lid.visual(
            Box((0.050, 0.024, 0.018)),
            origin=Origin(xyz=(0.599, y_center, 0.002)),
            material=galvanized,
        )
        lid.visual(
            Cylinder(radius=0.007, length=0.022),
            origin=Origin(xyz=(0.632, y_center, 0.008), rpy=(pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"hasp_knuckle_{'right' if y_center > 0.0 else 'left'}",
        )

    hasp_arm = model.part("hasp_arm")
    hasp_arm.inertial = Inertial.from_geometry(
        Box((0.014, 0.042, 0.062)),
        mass=1.1,
        origin=Origin(xyz=(0.003, 0.0, -0.028)),
    )
    hasp_arm.visual(
        Cylinder(radius=0.007, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="hasp_barrel",
    )
    hasp_arm.visual(
        Box((0.008, 0.010, 0.014)),
        origin=Origin(xyz=(0.003, -0.012, -0.010)),
        material=galvanized,
        name="hasp_lug_left",
    )
    hasp_arm.visual(
        Box((0.008, 0.010, 0.014)),
        origin=Origin(xyz=(0.003, 0.012, -0.010)),
        material=galvanized,
        name="hasp_lug_right",
    )
    hasp_slot_profile = [(x + 0.014, y) for x, y in rounded_rect_profile(0.026, 0.018, 0.004)]
    hasp_plate_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.060, 0.036, 0.006),
        [hasp_slot_profile],
        height=0.006,
        center=True,
    ).rotate_y(pi / 2.0)
    hasp_arm.visual(
        _mesh("hasp_plate", hasp_plate_geom),
        origin=Origin(xyz=(0.003, 0.0, -0.032)),
        material=galvanized,
        name="hasp_plate",
    )

    staple = model.part("staple")
    staple.inertial = Inertial.from_geometry(
        Box((0.016, 0.052, 0.034)),
        mass=0.5,
        origin=Origin(xyz=(0.004, 0.0, -0.014)),
    )
    staple.visual(
        Box((0.008, 0.014, 0.010)),
        origin=Origin(xyz=(0.004, -0.018, -0.020)),
        material=powder_coat,
        name="staple_pad_left",
    )
    staple.visual(
        Box((0.008, 0.014, 0.010)),
        origin=Origin(xyz=(0.004, 0.018, -0.020)),
        material=powder_coat,
        name="staple_pad_right",
    )
    staple_loop = wire_from_points(
        [
            (0.001, -0.018, -0.020),
            (0.008, -0.018, -0.020),
            (0.008, -0.018, -0.004),
            (0.008, 0.018, -0.004),
            (0.008, 0.018, -0.020),
            (0.001, 0.018, -0.020),
        ],
        radius=0.003,
        closed_path=False,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.005,
        corner_segments=6,
    )
    staple.visual(
        _mesh("staple_loop", staple_loop),
        material=galvanized,
        name="staple_loop",
    )

    model.articulation(
        "housing_to_lid",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lid,
        origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2, lower=0.0, upper=1.35),
    )
    model.articulation(
        "housing_to_staple",
        ArticulationType.FIXED,
        parent=housing,
        child=staple,
        origin=Origin(xyz=(outer * 0.5 - 0.001, 0.0, -0.006)),
    )
    model.articulation(
        "lid_to_hasp_arm",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=hasp_arm,
        origin=Origin(xyz=(0.632, 0.0, 0.008)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.52),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    lid = object_model.get_part("lid")
    staple = object_model.get_part("staple")
    hasp_arm = object_model.get_part("hasp_arm")
    lid_hinge = object_model.get_articulation("housing_to_lid")
    hasp_hinge = object_model.get_articulation("lid_to_hasp_arm")
    hasp_open = 1.45
    if hasp_hinge.motion_limits is not None and hasp_hinge.motion_limits.upper is not None:
        hasp_open = hasp_hinge.motion_limits.upper

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        hasp_arm,
        staple,
        elem_a="hasp_plate",
        elem_b="staple_loop",
        reason="The locked hasp is authored as a simplified slotted plate proxy that intentionally nests around the staple loop.",
    )

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

    ctx.expect_contact(
        staple,
        housing,
        elem_a="staple_pad_left",
        elem_b="front_wall",
        name="staple mounting pad is welded to the housing front wall",
    )

    with ctx.pose({lid_hinge: 0.0, hasp_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            housing,
            axis="z",
            positive_elem="lid_front_band",
            negative_elem="front_frame",
            min_gap=0.004,
            max_gap=0.015,
            name="closed lid sits just above the front frame",
        )
        ctx.expect_overlap(
            hasp_arm,
            staple,
            axes="yz",
            elem_a="hasp_plate",
            elem_b="staple_loop",
            min_overlap=0.012,
            name="closed hasp plate aligns around the staple loop",
        )

    with ctx.pose({lid_hinge: 0.0, hasp_hinge: hasp_open}):
        ctx.expect_gap(
            hasp_arm,
            staple,
            axis="z",
            positive_elem="hasp_plate",
            negative_elem="staple_loop",
            min_gap=0.012,
            name="opened hasp lifts clear of the staple",
        )

    with ctx.pose({lid_hinge: 1.15, hasp_hinge: hasp_open}):
        ctx.expect_gap(
            lid,
            housing,
            axis="z",
            positive_elem="lid_front_band",
            negative_elem="front_frame",
            min_gap=0.18,
            name="opened lid front edge swings well above the frame",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
