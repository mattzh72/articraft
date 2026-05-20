from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bladeless_ring_ceiling_fan")

    mount_white = model.material("mount_white", rgba=(0.95, 0.95, 0.93, 1.0))
    rotor_graphite = model.material("rotor_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.75, 0.77, 0.80, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.26, 0.27, 0.30, 1.0))

    def save_mesh(name: str, geometry: MeshGeometry):
        return mesh_from_geometry(geometry, name)

    canopy_profile = [
        (0.0, 0.000),
        (0.100, 0.000),
        (0.096, -0.010),
        (0.084, -0.036),
        (0.068, -0.067),
        (0.052, -0.093),
        (0.041, -0.106),
        (0.038, -0.112),
        (0.0, -0.112),
    ]
    canopy_geom = LatheGeometry(canopy_profile, segments=72)

    stationary = model.part("stationary_assembly")
    stationary.visual(
        save_mesh("ceiling_canopy_shell", canopy_geom),
        material=mount_white,
        name="ceiling_canopy",
    )
    stationary.visual(
        Cylinder(radius=0.038, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.120)),
        material=mount_white,
        name="lower_mount_collar",
    )
    stationary.visual(
        Cylinder(radius=0.014, length=0.380),
        origin=Origin(xyz=(0.0, 0.0, -0.319)),
        material=mount_white,
        name="downrod",
    )
    stationary.visual(
        Cylinder(radius=0.060, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.523)),
        material=dark_trim,
        name="stator_hub",
    )
    stationary.visual(
        Cylinder(radius=0.108, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.524)),
        material=brushed_metal,
        name="bearing_flange",
    )
    decorative_cover_geom = TorusGeometry(
        radius=0.044,
        tube=0.006,
        radial_segments=16,
        tubular_segments=56,
    ).translate(0.0, 0.0, -0.120)
    stationary.visual(
        save_mesh("decorative_cover_band", decorative_cover_geom),
        material=brushed_metal,
        name="decorative_ring_cover",
    )
    stationary.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.63),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, -0.315)),
    )

    rotor_ring = model.part("rotor_ring")
    rotor_ring.visual(
        save_mesh(
            "rotor_torus",
            TorusGeometry(
                radius=0.565,
                tube=0.052,
                radial_segments=20,
                tubular_segments=96,
            ),
        ),
        material=rotor_graphite,
        name="rotor_torus",
    )

    rotor_ring.visual(
        save_mesh(
            "rotor_bearing_cap",
            LatheGeometry.from_shell_profiles(
                [(0.106, -0.012), (0.106, 0.000)],
                [(0.064, -0.012), (0.064, 0.000)],
                segments=72,
                start_cap="flat",
                end_cap="flat",
                lip_samples=6,
            ),
        ),
        material=dark_trim,
        name="rotor_bearing_cap",
    )
    rotor_ring.visual(
        save_mesh(
            "rotor_hub_shell",
            LatheGeometry.from_shell_profiles(
                [(0.128, -0.078), (0.128, -0.010)],
                [(0.070, -0.078), (0.070, -0.010)],
                segments=72,
                start_cap="flat",
                end_cap="flat",
                lip_samples=6,
            ),
        ),
        material=dark_trim,
        name="rotor_hub_shell",
    )

    strut_profile = [
        (0.128, 0.0, -0.040),
        (0.250, 0.0, -0.014),
        (0.400, 0.0, -0.006),
        (0.520, 0.0, 0.000),
    ]
    strut_base = tube_from_spline_points(
        strut_profile,
        radius=0.012,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    spoke_geom = MeshGeometry()
    for index in range(4):
        spoke_geom.merge(strut_base.copy().rotate_z(index * (math.tau / 4.0)))
    rotor_ring.visual(
        save_mesh("rotor_spokes", spoke_geom),
        material=dark_trim,
        name="rotor_spokes",
    )
    rotor_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.62, length=0.11),
        mass=5.0,
        origin=Origin(),
    )

    model.articulation(
        "hub_to_rotor_ring",
        ArticulationType.CONTINUOUS,
        parent=stationary,
        child=rotor_ring,
        origin=Origin(xyz=(0.0, 0.0, -0.530)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=9.0),
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

    stationary = object_model.get_part("stationary_assembly")
    rotor_ring = object_model.get_part("rotor_ring")
    rotor_joint = object_model.get_articulation("hub_to_rotor_ring")
    decorative_cover = stationary.get_visual("decorative_ring_cover")

    limits = rotor_joint.motion_limits
    ctx.check(
        "rotor articulation is vertical continuous spin",
        rotor_joint.articulation_type == ArticulationType.CONTINUOUS
        and rotor_joint.axis == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None
        and limits.velocity >= 5.0,
        details=(
            f"type={rotor_joint.articulation_type}, axis={rotor_joint.axis}, "
            f"limits={limits}"
        ),
    )

    ctx.expect_origin_gap(
        stationary,
        rotor_ring,
        axis="z",
        min_gap=0.52,
        max_gap=0.60,
        name="rotor hangs below the ceiling mount",
    )

    with ctx.pose({rotor_joint: 0.0}):
        ctx.check(
            "stationary assembly includes decorative ring cover",
            decorative_cover is not None,
            details=f"visual={decorative_cover}",
        )
        ctx.expect_origin_distance(
            stationary,
            rotor_ring,
            axes="xy",
            max_dist=1e-6,
            name="rotor is concentric with the downrod axis",
        )
        ctx.expect_within(
            stationary,
            rotor_ring,
            axes="xy",
            inner_elem="stator_hub",
            outer_elem="rotor_hub_shell",
            margin=0.0,
            name="stationary motor hub nests inside the rotor hub shell",
        )
        ctx.expect_contact(
            rotor_ring,
            stationary,
            elem_a="rotor_bearing_cap",
            elem_b="bearing_flange",
            contact_tol=5e-5,
            name="rotor bearing cap rides on the stationary bearing flange",
        )
        ctx.expect_overlap(
            rotor_ring,
            stationary,
            axes="xy",
            elem_a="rotor_bearing_cap",
            elem_b="bearing_flange",
            min_overlap=0.20,
            name="bearing cap stays broadly engaged under the flange",
        )
        rest_aabb = ctx.part_world_aabb(rotor_ring)
        rest_size = None
        if rest_aabb is not None:
            rest_size = tuple(rest_aabb[1][i] - rest_aabb[0][i] for i in range(3))
        ctx.check(
            "rotor ring reads as a large thin hoop",
            rest_size is not None
            and rest_size[0] > 1.20
            and rest_size[1] > 1.20
            and rest_size[2] < 0.14,
            details=f"rotor_size={rest_size}",
        )
        rest_pos = ctx.part_world_position(rotor_ring)

    with ctx.pose({rotor_joint: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(rotor_ring)
        ctx.expect_within(
            stationary,
            rotor_ring,
            axes="xy",
            inner_elem="stator_hub",
            outer_elem="rotor_hub_shell",
            margin=0.0,
            name="rotor hub shell stays concentric while spinning",
        )

    ctx.check(
        "rotor spins in place about the vertical hub axis",
        rest_pos is not None
        and turned_pos is not None
        and all(abs(rest_pos[i] - turned_pos[i]) <= 1e-6 for i in range(3)),
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
