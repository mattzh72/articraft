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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
)


def _xy_section(width: float, height: float, radius: float, z_pos: float) -> list[tuple[float, float, float]]:
    return [(x, y, z_pos) for x, y in rounded_rect_profile(width, height, radius, corner_segments=8)]


def _build_housing_shell():
    return section_loft(
        [
            _xy_section(0.152, 0.108, 0.015, 0.000),
            _xy_section(0.150, 0.106, 0.015, 0.016),
            _xy_section(0.142, 0.098, 0.013, 0.036),
        ]
    )


def _build_dial_ring():
    return ExtrudeWithHolesGeometry(
        superellipse_profile(0.090, 0.090, exponent=3.2, segments=64),
        [superellipse_profile(0.026, 0.026, exponent=2.4, segments=40)],
        0.014,
        center=True,
        cap=True,
    )


def _build_retainer_ring():
    return ExtrudeWithHolesGeometry(
        superellipse_profile(0.038, 0.038, exponent=2.6, segments=48),
        [superellipse_profile(0.022, 0.022, exponent=2.0, segments=32)],
        0.003,
        center=True,
        cap=True,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat_service_dial")

    plate_steel = model.material("plate_steel", rgba=(0.56, 0.59, 0.62, 1.0))
    housing_grey = model.material("housing_grey", rgba=(0.68, 0.69, 0.67, 1.0))
    cover_grey = model.material("cover_grey", rgba=(0.50, 0.52, 0.54, 1.0))
    dial_black = model.material("dial_black", rgba=(0.11, 0.12, 0.12, 1.0))
    shaft_steel = model.material("shaft_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    retainer_zinc = model.material("retainer_zinc", rgba=(0.83, 0.84, 0.80, 1.0))
    index_red = model.material("index_red", rgba=(0.74, 0.18, 0.12, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        Box((0.170, 0.126, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=plate_steel,
        name="wall_plate",
    )
    backplate.visual(
        Box((0.142, 0.094, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=plate_steel,
        name="standoff_frame",
    )
    backplate.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, 0.042, 0.007)),
        material=plate_steel,
        name="upper_mount_boss",
    )
    backplate.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, -0.042, 0.007)),
        material=plate_steel,
        name="lower_mount_boss",
    )
    backplate.inertial = Inertial.from_geometry(
        Box((0.170, 0.126, 0.010)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    housing = model.part("housing")
    housing.visual(
        mesh_from_geometry(_build_housing_shell(), "thermostat_housing_shell"),
        material=housing_grey,
        name="housing_shell",
    )
    housing.visual(
        Cylinder(radius=0.036, length=0.010),
        origin=Origin(xyz=(0.0, 0.013, 0.038)),
        material=housing_grey,
        name="dial_boss",
    )
    housing.visual(
        Box((0.118, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.045, 0.033)),
        material=housing_grey,
        name="top_brow",
    )
    housing.visual(
        Box((0.010, 0.026, 0.008)),
        origin=Origin(xyz=(-0.047, -0.024, 0.036)),
        material=housing_grey,
        name="service_left_jamb",
    )
    housing.visual(
        Box((0.010, 0.026, 0.008)),
        origin=Origin(xyz=(0.047, -0.024, 0.036)),
        material=housing_grey,
        name="service_right_jamb",
    )
    housing.visual(
        Box((0.104, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, -0.010, 0.036)),
        material=housing_grey,
        name="service_top_stop",
    )
    housing.visual(
        Box((0.022, 0.008, 0.008)),
        origin=Origin(xyz=(-0.026, -0.046, 0.034)),
        material=housing_grey,
        name="left_hinge_mount",
    )
    housing.visual(
        Box((0.022, 0.008, 0.008)),
        origin=Origin(xyz=(0.026, -0.046, 0.034)),
        material=housing_grey,
        name="right_hinge_mount",
    )
    housing.visual(
        Box((0.008, 0.010, 0.005)),
        origin=Origin(xyz=(0.0, 0.044, 0.0375)),
        material=index_red,
        name="index_fin",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.152, 0.108, 0.042)),
        mass=1.10,
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
    )

    service_cover = model.part("service_cover")
    service_cover.visual(
        Box((0.084, 0.022, 0.005)),
        origin=Origin(xyz=(0.0, 0.011, 0.0025)),
        material=cover_grey,
        name="cover_panel",
    )
    service_cover.visual(
        Cylinder(radius=0.004, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.0065), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cover_grey,
        name="hinge_barrel",
    )
    service_cover.visual(
        Box((0.010, 0.009, 0.006)),
        origin=Origin(xyz=(-0.022, 0.0045, 0.005)),
        material=cover_grey,
        name="left_hinge_strap",
    )
    service_cover.visual(
        Box((0.010, 0.009, 0.006)),
        origin=Origin(xyz=(0.022, 0.0045, 0.005)),
        material=cover_grey,
        name="right_hinge_strap",
    )
    service_cover.visual(
        Box((0.044, 0.004, 0.005)),
        origin=Origin(xyz=(0.0, 0.022, 0.004)),
        material=cover_grey,
        name="latch_rib",
    )
    service_cover.visual(
        Box((0.026, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.024, 0.007)),
        material=cover_grey,
        name="finger_pull",
    )
    service_cover.inertial = Inertial.from_geometry(
        Box((0.084, 0.028, 0.012)),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=shaft_steel,
        name="thrust_washer",
    )
    shaft.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=shaft_steel,
        name="center_shaft",
    )
    shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.024),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    retainer = model.part("retainer")
    retainer.visual(
        mesh_from_geometry(_build_retainer_ring(), "thermostat_retainer_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=retainer_zinc,
        name="retainer_flange",
    )
    retainer.visual(
        Box((0.005, 0.010, 0.003)),
        origin=Origin(xyz=(0.0105, 0.0, 0.0015)),
        material=retainer_zinc,
        name="retainer_left_clamp",
    )
    retainer.visual(
        Box((0.005, 0.010, 0.003)),
        origin=Origin(xyz=(-0.0105, 0.0, 0.0015)),
        material=retainer_zinc,
        name="retainer_right_clamp",
    )
    retainer.inertial = Inertial.from_geometry(
        Cylinder(radius=0.019, length=0.004),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(_build_dial_ring(), "thermostat_dial_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=dial_black,
        name="dial_ring",
    )
    dial.visual(
        Box((0.014, 0.010, 0.005)),
        origin=Origin(xyz=(0.0, 0.040, 0.019)),
        material=dial_black,
        name="dial_pointer",
    )
    dial.visual(
        Box((0.018, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, -0.041, 0.016)),
        material=dial_black,
        name="grip_pad",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.020),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    model.articulation(
        "backplate_to_housing",
        ArticulationType.FIXED,
        parent=backplate,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )
    model.articulation(
        "housing_to_service_cover",
        ArticulationType.FIXED,
        parent=housing,
        child=service_cover,
        origin=Origin(xyz=(0.0, -0.047, 0.040)),
    )
    model.articulation(
        "housing_to_shaft",
        ArticulationType.FIXED,
        parent=housing,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.010, 0.043)),
    )
    model.articulation(
        "shaft_to_retainer",
        ArticulationType.FIXED,
        parent=shaft,
        child=retainer,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )
    model.articulation(
        "shaft_to_dial",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=dial,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.5,
            lower=-2.35,
            upper=2.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    housing = object_model.get_part("housing")
    service_cover = object_model.get_part("service_cover")
    shaft = object_model.get_part("shaft")
    retainer = object_model.get_part("retainer")
    dial = object_model.get_part("dial")
    dial_joint = object_model.get_articulation("shaft_to_dial")

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

    ctx.expect_contact(housing, backplate, name="housing mounted to backplate")
    ctx.expect_contact(service_cover, housing, name="service cover seated on housing")
    ctx.expect_contact(shaft, housing, name="shaft anchored in housing boss")
    ctx.expect_contact(dial, shaft, name="dial rides on shaft washer")
    ctx.expect_contact(retainer, dial, name="retainer captures dial")
    ctx.expect_origin_distance(dial, shaft, axes="xy", max_dist=0.0005, name="dial remains centered on shaft")
    ctx.expect_gap(dial, housing, axis="z", min_gap=0.004, max_gap=0.014, name="dial stands proud of housing")
    ctx.expect_overlap(dial, shaft, axes="xy", min_overlap=0.016, name="dial hole remains centered over shaft")
    ctx.expect_within(service_cover, housing, axes="xy", margin=0.006, name="service cover stays inside housing envelope")

    dial_axis_position = ctx.part_world_position(shaft)
    with ctx.pose({dial_joint: 0.0}):
        pointer_aabb_rest = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    with ctx.pose({dial_joint: math.pi / 2.0}):
        pointer_aabb_quarter = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    with ctx.pose({dial_joint: dial_joint.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps at dial upper stop")

    pointer_center_rest = _aabb_center(pointer_aabb_rest)
    pointer_center_quarter = _aabb_center(pointer_aabb_quarter)
    pointer_rotates = (
        dial_axis_position is not None
        and pointer_center_rest is not None
        and pointer_center_quarter is not None
        and abs(pointer_center_rest[0] - dial_axis_position[0]) < 0.008
        and (pointer_center_rest[1] - dial_axis_position[1]) > 0.028
        and (pointer_center_quarter[0] - dial_axis_position[0]) < -0.028
        and abs(pointer_center_quarter[1] - dial_axis_position[1]) < 0.010
    )
    ctx.check(
        "dial pointer sweeps around shaft",
        pointer_rotates,
        details=(
            f"shaft={dial_axis_position}, rest={pointer_center_rest}, quarter={pointer_center_quarter}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
