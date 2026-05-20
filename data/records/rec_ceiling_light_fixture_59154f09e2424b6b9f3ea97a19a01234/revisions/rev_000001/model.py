from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_ceiling_light")

    satin_nickel = model.material("satin_nickel", rgba=(0.72, 0.70, 0.66, 1.0))
    ceiling_white = model.material("ceiling_white", rgba=(0.92, 0.91, 0.86, 1.0))
    inner_white = model.material("warm_reflector_white", rgba=(0.98, 0.94, 0.84, 1.0))
    shadow = model.material("shadow_gap", rgba=(0.015, 0.014, 0.012, 1.0))
    screw_dark = model.material("dark_screw_heads", rgba=(0.10, 0.095, 0.085, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.88, 0.96, 1.0, 0.58))

    fixture = model.part("ceiling_mount")

    fixture.visual(
        Cylinder(radius=0.245, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=ceiling_white,
        name="ceiling_plate",
    )

    canopy_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.085, -0.008),
            (0.150, -0.010),
            (0.178, -0.026),
            (0.190, -0.060),
            (0.182, -0.073),
        ],
        inner_profile=[
            (0.045, -0.012),
            (0.120, -0.018),
            (0.146, -0.033),
            (0.154, -0.061),
            (0.150, -0.069),
        ],
        segments=96,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )
    fixture.visual(
        mesh_from_geometry(canopy_shell, "canopy_shell"),
        material=satin_nickel,
        name="canopy_shell",
    )

    fixture.visual(
        Cylinder(radius=0.076, length=0.098),
        origin=Origin(xyz=(0.0, 0.0, -0.053)),
        material=satin_nickel,
        name="center_boss",
    )

    housing_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.180, -0.062),
            (0.225, -0.082),
            (0.258, -0.126),
            (0.282, -0.158),
            (0.292, -0.166),
        ],
        inner_profile=[
            (0.145, -0.070),
            (0.182, -0.090),
            (0.212, -0.130),
            (0.225, -0.158),
            (0.224, -0.166),
        ],
        segments=128,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )
    fixture.visual(
        mesh_from_geometry(housing_shell, "layered_housing_shell"),
        material=satin_nickel,
        name="layered_housing",
    )

    lower_bezel = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.276, -0.154),
            (0.304, -0.162),
            (0.312, -0.171),
            (0.302, -0.176),
        ],
        inner_profile=[
            (0.222, -0.156),
            (0.218, -0.164),
            (0.219, -0.173),
            (0.226, -0.176),
        ],
        segments=128,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )
    fixture.visual(
        mesh_from_geometry(lower_bezel, "lower_service_bezel"),
        material=satin_nickel,
        name="lower_bezel",
    )

    reflector_lip = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.210, -0.116),
            (0.215, -0.148),
            (0.211, -0.158),
        ],
        inner_profile=[
            (0.155, -0.124),
            (0.180, -0.150),
            (0.188, -0.158),
        ],
        segments=96,
        start_cap="flat",
        end_cap="round",
        lip_samples=6,
    )
    fixture.visual(
        mesh_from_geometry(reflector_lip, "white_reflector_lip"),
        material=inner_white,
        name="reflector_lip",
    )

    fixture.visual(
        Cylinder(radius=0.142, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.102)),
        material=inner_white,
        name="driver_tray",
    )
    fixture.visual(
        Cylinder(radius=0.118, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.151)),
        material=Material("warm_led_board", rgba=(1.0, 0.86, 0.42, 1.0)),
        name="led_board",
    )

    for idx, angle in enumerate((math.radians(20), math.radians(140), math.radians(260))):
        x = 0.095 * math.cos(angle)
        y = 0.095 * math.sin(angle)
        fixture.visual(
            Cylinder(radius=0.007, length=0.049),
            origin=Origin(xyz=(x, y, -0.1265)),
            material=satin_nickel,
            name=f"tray_standoff_{idx}",
        )

    for idx, radius in enumerate((0.233, 0.263)):
        groove = TorusGeometry(radius=radius, tube=0.0022, radial_segments=12, tubular_segments=96)
        fixture.visual(
            mesh_from_geometry(groove, f"shadow_reveal_{idx}"),
            origin=Origin(xyz=(0.0, 0.0, -0.092 - idx * 0.040)),
            material=shadow,
            name=f"shadow_reveal_{idx}",
        )

    for idx, angle in enumerate((math.radians(45), math.radians(135), math.radians(225), math.radians(315))):
        x = 0.253 * math.cos(angle)
        y = 0.253 * math.sin(angle)
        fixture.visual(
            Cylinder(radius=0.008, length=0.003),
            origin=Origin(xyz=(x, y, -0.1765)),
            material=screw_dark,
            name=f"bezel_screw_{idx}",
        )

    hinge_axis_y = 0.328
    hinge_axis_z = -0.181
    fixture.visual(
        Box((0.410, 0.032, 0.006)),
        origin=Origin(xyz=(0.0, hinge_axis_y - 0.032, hinge_axis_z + 0.006)),
        material=satin_nickel,
        name="hinge_mount_rail",
    )
    for idx, x in enumerate((-0.150, 0.0, 0.150)):
        fixture.visual(
            Box((0.064, 0.040, 0.010)),
            origin=Origin(xyz=(x, hinge_axis_y - 0.012, hinge_axis_z + 0.003)),
            material=satin_nickel,
            name=f"fixed_hinge_leaf_{idx}",
        )
        fixture.visual(
            Cylinder(radius=0.008, length=0.055),
            origin=Origin(xyz=(x, hinge_axis_y, hinge_axis_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_nickel,
            name=f"fixed_hinge_knuckle_{idx}",
        )
    for idx, x in enumerate((-0.184, 0.184)):
        fixture.visual(
            Sphere(radius=0.009),
            origin=Origin(xyz=(x, hinge_axis_y, hinge_axis_z)),
            material=satin_nickel,
            name=f"hinge_pin_cap_{idx}",
        )

    fixture.visual(
        Box((0.098, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, -0.243, -0.178)),
        material=satin_nickel,
        name="latch_receiver",
    )
    fixture.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(xyz=(0.0, -0.256, -0.178), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=screw_dark,
        name="latch_roller",
    )

    access = model.part("access_lens")
    lens_retainer = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.204, -0.006),
            (0.218, -0.004),
            (0.222, -0.010),
            (0.215, -0.016),
            (0.203, -0.014),
        ],
        inner_profile=[
            (0.184, -0.006),
            (0.186, -0.008),
            (0.186, -0.014),
            (0.182, -0.016),
            (0.180, -0.010),
        ],
        segments=128,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )
    access.visual(
        mesh_from_geometry(lens_retainer, "access_lens_retainer"),
        origin=Origin(xyz=(0.0, -0.328, 0.0)),
        material=satin_nickel,
        name="lens_retainer",
    )
    access.visual(
        Cylinder(radius=0.181, length=0.007),
        origin=Origin(xyz=(0.0, -0.328, -0.011)),
        material=frosted_glass,
        name="diffuser",
    )
    for idx, x in enumerate((-0.075, 0.075)):
        access.visual(
            Box((0.074, 0.115, 0.006)),
            origin=Origin(xyz=(x, -0.060, -0.004)),
            material=satin_nickel,
            name=f"moving_hinge_leaf_{idx}",
        )
    for idx, x in enumerate((-0.075, 0.075)):
        access.visual(
            Cylinder(radius=0.008, length=0.075),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_nickel,
            name=f"moving_hinge_knuckle_{idx}",
        )
    access.visual(
        Box((0.058, 0.028, 0.012)),
        origin=Origin(xyz=(0.0, -0.545, -0.004)),
        material=satin_nickel,
        name="spring_latch_tab",
    )
    access.visual(
        Cylinder(radius=0.010, length=0.003),
        origin=Origin(xyz=(0.0, -0.559, -0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=screw_dark,
        name="latch_release_dimple",
    )

    model.articulation(
        "lens_hinge",
        ArticulationType.REVOLUTE,
        parent=fixture,
        child=access,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.5, velocity=1.6, lower=0.0, upper=1.35),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    fixture = object_model.get_part("ceiling_mount")
    access = object_model.get_part("access_lens")
    hinge = object_model.get_articulation("lens_hinge")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_within(
            access,
            fixture,
            axes="xy",
            inner_elem="lens_retainer",
            outer_elem="lower_bezel",
            margin=0.008,
            name="closed lens retainer nests inside lower bezel",
        )
        ctx.expect_gap(
            fixture,
            access,
            axis="z",
            positive_elem="lower_bezel",
            negative_elem="lens_retainer",
            min_gap=0.002,
            max_gap=0.012,
            name="closed access ring sits just below the service bezel",
        )
        closed_aabb = ctx.part_world_aabb(access)

    with ctx.pose({hinge: 1.35}):
        open_aabb = ctx.part_world_aabb(access)
        ctx.expect_gap(
            fixture,
            access,
            axis="z",
            positive_elem="lower_bezel",
            negative_elem="diffuser",
            min_gap=0.035,
            name="opened diffuser swings down clear of the trim",
        )

    closed_min_z = closed_aabb[0][2] if closed_aabb is not None else None
    open_min_z = open_aabb[0][2] if open_aabb is not None else None
    ctx.check(
        "hinged access lens opens downward",
        closed_min_z is not None and open_min_z is not None and open_min_z < closed_min_z - 0.18,
        details=f"closed_min_z={closed_min_z}, open_min_z={open_min_z}",
    )

    return ctx.report()


object_model = build_object_model()
