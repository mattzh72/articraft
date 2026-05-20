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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _side_cheek_profile() -> list[tuple[float, float]]:
    return [
        (-0.56, 0.18),
        (-0.38, 0.19),
        (-0.08, 0.20),
        (0.24, 0.22),
        (0.54, 0.25),
        (0.54, 0.34),
        (0.36, 0.39),
        (0.08, 0.44),
        (-0.18, 0.48),
        (-0.42, 0.51),
        (-0.56, 0.47),
    ]


def _build_cheek_mesh(name: str, side_y: float, thickness: float):
    cheek = ExtrudeGeometry(
        _side_cheek_profile(),
        thickness,
        center=True,
        cap=True,
        closed=True,
    )
    cheek.rotate_x(math.pi / 2.0).translate(0.0, side_y, 0.0)
    return _save_mesh(name, cheek)


def _build_quoin_mesh(name: str, width: float):
    profile = [
        (-0.14, 0.0),
        (0.14, 0.0),
        (0.14, 0.040),
        (-0.14, 0.012),
    ]
    geom = ExtrudeGeometry(
        profile,
        width,
        center=True,
        cap=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return _save_mesh(name, geom)


def _build_barrel_body_mesh(name: str):
    profile = [
        (0.0, -0.35),
        (0.055, -0.35),
        (0.075, -0.33),
        (0.110, -0.29),
        (0.148, -0.24),
        (0.180, -0.17),
        (0.194, -0.04),
        (0.188, 0.18),
        (0.175, 0.46),
        (0.164, 0.66),
        (0.176, 0.72),
        (0.166, 0.79),
        (0.130, 0.83),
        (0.0, 0.83),
    ]
    geom = LatheGeometry(profile, segments=72, closed=True)
    geom.rotate_y(math.pi / 2.0)
    return _save_mesh(name, geom)


def _add_wheel(
    model: ArticulatedObject,
    *,
    part_name: str,
    joint_name: str,
    carriage,
    wood,
    iron,
    wheel_x: float,
    wheel_y: float,
    wheel_z: float,
) -> None:
    wheel = model.part(part_name)
    wheel.visual(
        Cylinder(radius=0.123, length=0.068),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wood,
        name="wheel_disc",
    )
    wheel.visual(
        Cylinder(radius=0.038, length=0.080),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wood,
        name="hub_boss",
    )
    wheel.visual(
        Cylinder(radius=0.128, length=0.058),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="iron_tyre",
    )
    wheel.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle_cap",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.128, length=0.080),
        mass=8.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        joint_name,
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wheel,
        origin=Origin(xyz=(wheel_x, wheel_y, wheel_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=6.0,
            lower=-12.0,
            upper=12.0,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="naval_truck_cannon")

    oak_dark = model.material("oak_dark", rgba=(0.42, 0.28, 0.16, 1.0))
    oak_weathered = model.material("oak_weathered", rgba=(0.55, 0.40, 0.24, 1.0))
    iron_black = model.material("iron_black", rgba=(0.16, 0.17, 0.18, 1.0))
    iron_band = model.material("iron_band", rgba=(0.26, 0.27, 0.28, 1.0))
    bronze_oil = model.material("bronze_oil", rgba=(0.23, 0.18, 0.11, 1.0))

    cheek_thickness = 0.05
    cheek_center_y = 0.36
    wheel_center_y = 0.465
    axle_stub_center_y = 0.385
    wheel_radius = 0.128
    wheel_z = wheel_radius
    trunnion_height = 0.46

    carriage = model.part("carriage")
    carriage.visual(
        _build_cheek_mesh("left_carriage_cheek", cheek_center_y, cheek_thickness),
        material=oak_dark,
        name="left_cheek",
    )
    carriage.visual(
        _build_cheek_mesh("right_carriage_cheek", -cheek_center_y, cheek_thickness),
        material=oak_dark,
        name="right_cheek",
    )
    carriage.visual(
        Box((0.18, 0.70, 0.06)),
        origin=Origin(xyz=(0.40, 0.0, 0.16)),
        material=oak_dark,
        name="front_axle_bed",
    )
    carriage.visual(
        Box((0.18, 0.70, 0.06)),
        origin=Origin(xyz=(-0.43, 0.0, 0.16)),
        material=oak_dark,
        name="rear_axle_bed",
    )
    carriage.visual(
        Box((0.80, 0.34, 0.02)),
        origin=Origin(xyz=(-0.02, 0.0, 0.20)),
        material=oak_weathered,
        name="bed_plank",
    )
    carriage.visual(
        Box((0.46, 0.18, 0.016)),
        origin=Origin(xyz=(-0.10, 0.0, 0.218)),
        material=oak_weathered,
        name="quoin_slide",
    )
    carriage.visual(
        Box((0.14, 0.72, 0.05)),
        origin=Origin(xyz=(0.40, 0.0, 0.185)),
        material=oak_dark,
        name="front_transom",
    )
    carriage.visual(
        Box((0.12, 0.58, 0.20)),
        origin=Origin(xyz=(-0.46, 0.0, 0.29)),
        material=oak_dark,
        name="rear_transom",
    )
    carriage.visual(
        Box((0.12, 0.12, 0.05)),
        origin=Origin(xyz=(0.00, 0.305, trunnion_height - 0.063)),
        material=oak_dark,
        name="left_trunnion_saddle",
    )
    carriage.visual(
        Box((0.12, 0.12, 0.05)),
        origin=Origin(xyz=(0.00, -0.305, trunnion_height - 0.063)),
        material=oak_dark,
        name="right_trunnion_saddle",
    )

    for stub_name, stub_x, stub_y in (
        ("front_left_axle_stub", 0.40, axle_stub_center_y),
        ("front_right_axle_stub", 0.40, -axle_stub_center_y),
        ("rear_left_axle_stub", -0.43, axle_stub_center_y),
        ("rear_right_axle_stub", -0.43, -axle_stub_center_y),
    ):
        carriage.visual(
            Cylinder(radius=0.022, length=0.080),
            origin=Origin(
                xyz=(stub_x, stub_y, wheel_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=iron_band,
            name=stub_name,
        )

    carriage.inertial = Inertial.from_geometry(
        Box((1.20, 0.90, 0.42)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
    )

    barrel = model.part("barrel")
    barrel.visual(
        _build_barrel_body_mesh("cast_iron_barrel_body"),
        material=iron_black,
        name="barrel_body",
    )
    barrel.visual(
        Cylinder(radius=0.038, length=0.140),
        origin=Origin(xyz=(0.0, 0.250, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron_black,
        name="left_trunnion",
    )
    barrel.visual(
        Cylinder(radius=0.038, length=0.140),
        origin=Origin(xyz=(0.0, -0.250, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron_black,
        name="right_trunnion",
    )
    barrel.visual(
        Cylinder(radius=0.178, length=0.030),
        origin=Origin(xyz=(0.72, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bronze_oil,
        name="muzzle_ring",
    )
    barrel.visual(
        Cylinder(radius=0.198, length=0.050),
        origin=Origin(xyz=(-0.08, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_band,
        name="breech_ring",
    )
    barrel.visual(
        Sphere(radius=0.055),
        origin=Origin(xyz=(-0.33, 0.0, 0.0)),
        material=iron_band,
        name="cascabel",
    )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.195, length=1.18),
        mass=720.0,
        origin=Origin(xyz=(0.24, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "barrel_elevation",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(0.0, 0.0, trunnion_height)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.8,
            lower=-0.16,
            upper=0.38,
        ),
    )

    quoin = model.part("quoin")
    quoin.visual(
        _build_quoin_mesh("wooden_quoin_wedge", 0.16),
        material=oak_weathered,
        name="quoin_wedge",
    )
    quoin.visual(
        Cylinder(radius=0.012, length=0.18),
        origin=Origin(xyz=(-0.11, 0.0, 0.016), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron_band,
        name="quoin_handle",
    )
    quoin.inertial = Inertial.from_geometry(
        Box((0.28, 0.18, 0.05)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    model.articulation(
        "quoin_adjustment",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=quoin,
        origin=Origin(xyz=(-0.18, 0.0, 0.226)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.10,
            lower=0.0,
            upper=0.18,
        ),
    )

    _add_wheel(
        model,
        part_name="front_left_wheel",
        joint_name="front_left_axle",
        carriage=carriage,
        wood=oak_weathered,
        iron=iron_band,
        wheel_x=0.40,
        wheel_y=wheel_center_y,
        wheel_z=wheel_z,
    )
    _add_wheel(
        model,
        part_name="front_right_wheel",
        joint_name="front_right_axle",
        carriage=carriage,
        wood=oak_weathered,
        iron=iron_band,
        wheel_x=0.40,
        wheel_y=-wheel_center_y,
        wheel_z=wheel_z,
    )
    _add_wheel(
        model,
        part_name="rear_left_wheel",
        joint_name="rear_left_axle",
        carriage=carriage,
        wood=oak_weathered,
        iron=iron_band,
        wheel_x=-0.43,
        wheel_y=wheel_center_y,
        wheel_z=wheel_z,
    )
    _add_wheel(
        model,
        part_name="rear_right_wheel",
        joint_name="rear_right_axle",
        carriage=carriage,
        wood=oak_weathered,
        iron=iron_band,
        wheel_x=-0.43,
        wheel_y=-wheel_center_y,
        wheel_z=wheel_z,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    quoin = object_model.get_part("quoin")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    barrel_elevation = object_model.get_articulation("barrel_elevation")
    quoin_adjustment = object_model.get_articulation("quoin_adjustment")
    front_left_axle = object_model.get_articulation("front_left_axle")
    front_right_axle = object_model.get_articulation("front_right_axle")
    rear_left_axle = object_model.get_articulation("rear_left_axle")
    rear_right_axle = object_model.get_articulation("rear_right_axle")

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

    ctx.expect_contact(
        barrel,
        carriage,
        elem_a="left_trunnion",
        elem_b="left_trunnion_saddle",
        contact_tol=0.0015,
        name="left trunnion is seated on the port cheek saddle",
    )
    ctx.expect_contact(
        barrel,
        carriage,
        elem_a="right_trunnion",
        elem_b="right_trunnion_saddle",
        contact_tol=0.0015,
        name="right trunnion is seated on the starboard cheek saddle",
    )
    ctx.expect_contact(
        quoin,
        carriage,
        elem_a="quoin_wedge",
        elem_b="quoin_slide",
        contact_tol=0.0015,
        name="quoin wedge bears on the carriage slide bed",
    )

    for wheel, stub_name, check_name in (
        (front_left_wheel, "front_left_axle_stub", "front left wheel contacts its axle stub"),
        (front_right_wheel, "front_right_axle_stub", "front right wheel contacts its axle stub"),
        (rear_left_wheel, "rear_left_axle_stub", "rear left wheel contacts its axle stub"),
        (rear_right_wheel, "rear_right_axle_stub", "rear right wheel contacts its axle stub"),
    ):
        ctx.expect_contact(
            wheel,
            carriage,
            elem_a="hub_boss",
            elem_b=stub_name,
            contact_tol=0.0015,
            name=check_name,
        )

    ctx.check(
        "barrel hinge axis pitches upward for positive motion",
        barrel_elevation.axis == (0.0, -1.0, 0.0),
        details=f"axis={barrel_elevation.axis}",
    )
    ctx.check(
        "quoin slides fore and aft along the carriage bed",
        quoin_adjustment.axis == (1.0, 0.0, 0.0),
        details=f"axis={quoin_adjustment.axis}",
    )
    for joint, name in (
        (front_left_axle, "front left axle spins on transverse axis"),
        (front_right_axle, "front right axle spins on transverse axis"),
        (rear_left_axle, "rear left axle spins on transverse axis"),
        (rear_right_axle, "rear right axle spins on transverse axis"),
    ):
        ctx.check(name, joint.axis == (0.0, 1.0, 0.0), details=f"axis={joint.axis}")

    muzzle_rest = ctx.part_element_world_aabb(barrel, elem="muzzle_ring")
    quoin_rest = ctx.part_world_position(quoin)
    barrel_upper = barrel_elevation.motion_limits.upper if barrel_elevation.motion_limits else None
    quoin_upper = quoin_adjustment.motion_limits.upper if quoin_adjustment.motion_limits else None

    with ctx.pose({barrel_elevation: 0.30}):
        muzzle_raised = ctx.part_element_world_aabb(barrel, elem="muzzle_ring")
        ctx.expect_contact(
            barrel,
            carriage,
            elem_a="left_trunnion",
            elem_b="left_trunnion_saddle",
            contact_tol=0.003,
            name="left trunnion remains supported while elevating",
        )
        ctx.expect_contact(
            barrel,
            carriage,
            elem_a="right_trunnion",
            elem_b="right_trunnion_saddle",
            contact_tol=0.003,
            name="right trunnion remains supported while elevating",
        )

    ctx.check(
        "positive barrel elevation raises the muzzle",
        muzzle_rest is not None
        and muzzle_raised is not None
        and ((muzzle_raised[0][2] + muzzle_raised[1][2]) * 0.5)
        > ((muzzle_rest[0][2] + muzzle_rest[1][2]) * 0.5) + 0.06,
        details=f"rest={muzzle_rest}, raised={muzzle_raised}",
    )

    if quoin_upper is not None:
        with ctx.pose({quoin_adjustment: quoin_upper}):
            quoin_extended = ctx.part_world_position(quoin)
            ctx.expect_within(
                quoin,
                carriage,
                axes="y",
                inner_elem="quoin_wedge",
                outer_elem="quoin_slide",
                margin=0.0,
                name="quoin remains between the carriage cheeks when extended",
            )
            ctx.expect_overlap(
                quoin,
                carriage,
                axes="x",
                elem_a="quoin_wedge",
                elem_b="quoin_slide",
                min_overlap=0.18,
                name="quoin stays retained on the slide bed at full extension",
            )
        ctx.check(
            "quoin advances forward when adjusted",
            quoin_rest is not None
            and quoin_extended is not None
            and quoin_extended[0] > quoin_rest[0] + 0.10,
            details=f"rest={quoin_rest}, extended={quoin_extended}",
        )

    if barrel_upper is not None:
        ctx.check(
            "barrel elevation range is plausible for a naval truck carriage",
            barrel_upper >= 0.30,
            details=f"upper={barrel_upper}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
