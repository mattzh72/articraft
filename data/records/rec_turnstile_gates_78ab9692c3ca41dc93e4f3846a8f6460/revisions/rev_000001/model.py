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
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, name)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _arm_tube_geometry() -> MeshGeometry:
    return tube_from_spline_points(
        [
            (0.11, 0.0, 0.100),
            (0.34, 0.0, 0.100),
            (0.58, 0.0, 0.098),
        ],
        radius=0.017,
        samples_per_segment=10,
        radial_segments=18,
        cap_ends=True,
    )


def _hub_shell_geometry() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.068, 0.000),
            (0.070, 0.010),
            (0.070, 0.038),
            (0.061, 0.052),
            (0.058, 0.134),
            (0.063, 0.162),
            (0.059, 0.184),
        ],
        [
            (0.030, 0.000),
            (0.030, 0.034),
            (0.028, 0.050),
            (0.028, 0.132),
            (0.029, 0.160),
            (0.029, 0.184),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _weather_hood_geometry() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.078, 0.000),
            (0.126, 0.004),
            (0.176, 0.012),
            (0.214, 0.024),
            (0.226, 0.034),
        ],
        [
            (0.072, -0.001),
            (0.120, 0.001),
            (0.170, 0.008),
            (0.208, 0.020),
            (0.218, 0.028),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_turnstile_gate")

    powder_coat = model.material("powder_coat", rgba=(0.32, 0.35, 0.38, 1.0))
    stainless = model.material("stainless", rgba=(0.73, 0.76, 0.79, 1.0))
    dark_seal = model.material("dark_seal", rgba=(0.10, 0.11, 0.12, 1.0))
    galvanized = model.material("galvanized", rgba=(0.62, 0.66, 0.70, 1.0))

    hood_mesh = _save_mesh("weather_hood", _weather_hood_geometry())
    hub_mesh = _save_mesh("turnstile_hub_shell", _hub_shell_geometry())
    arm_mesh = _save_mesh("turnstile_arm_tube", _arm_tube_geometry())

    frame = model.part("frame")
    frame.visual(
        Cylinder(radius=0.22, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=galvanized,
        name="base_flange",
    )
    frame.visual(
        Cylinder(radius=0.098, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=powder_coat,
        name="base_skirt",
    )
    frame.visual(
        Cylinder(radius=0.075, length=0.82),
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
        material=powder_coat,
        name="central_post",
    )
    frame.visual(
        Cylinder(radius=0.148, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.90)),
        material=powder_coat,
        name="mechanism_housing",
    )
    frame.visual(
        Cylinder(radius=0.056, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.985)),
        material=dark_seal,
        name="seal_neck",
    )
    frame.visual(
        Cylinder(radius=0.156, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.979)),
        material=dark_seal,
        name="hood_gasket",
    )
    frame.visual(
        hood_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.954)),
        material=powder_coat,
        name="weather_hood",
    )
    frame.visual(
        Cylinder(radius=0.072, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 1.000)),
        material=stainless,
        name="bearing_seat",
    )
    frame.visual(
        Cylinder(radius=0.017, length=0.226),
        origin=Origin(xyz=(0.0, 0.0, 1.125)),
        material=stainless,
        name="spindle_core",
    )
    frame.visual(
        Cylinder(radius=0.034, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 1.246)),
        material=stainless,
        name="spindle_cap",
    )
    for index in range(4):
        angle = (math.tau * index) / 4.0 + (math.pi / 4.0)
        x = math.cos(angle) * 0.135
        y = math.sin(angle) * 0.135
        frame.visual(
            Cylinder(radius=0.010, length=0.048),
            origin=Origin(xyz=(x, y, 0.024)),
            material=stainless,
            name=f"anchor_bolt_{index}",
        )
        frame.visual(
            Cylinder(radius=0.018, length=0.007),
            origin=Origin(xyz=(x, y, 0.0205)),
            material=galvanized,
            name=f"anchor_washer_{index}",
        )
    frame.inertial = Inertial.from_geometry(
        Box((0.46, 0.46, 1.26)),
        mass=78.0,
        origin=Origin(xyz=(0.0, 0.0, 0.63)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        hub_mesh,
        material=stainless,
        name="hub_shell",
    )
    for index in range(3):
        angle = (math.tau * index) / 3.0
        rotor.visual(
            Cylinder(radius=0.028, length=0.14),
            origin=Origin(
                xyz=(math.cos(angle) * 0.11, math.sin(angle) * 0.11, 0.100),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=stainless,
            name=f"arm_socket_{index}",
        )
        rotor.visual(
            arm_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=stainless,
            name=f"arm_{index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.60, length=0.22),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
    )

    model.articulation(
        "turnstile_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 1.012)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("turnstile_spin")

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

    ctx.check(
        "turnstile_joint_is_vertical_continuous",
        spin.joint_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        f"expected continuous vertical rotation, got type={spin.joint_type} axis={spin.axis}",
    )
    ctx.expect_contact(
        rotor,
        frame,
        elem_a="hub_shell",
        elem_b="bearing_seat",
        name="hub_shell_seats_on_bearing_ring",
    )
    ctx.expect_within(
        frame,
        rotor,
        axes="xy",
        inner_elem="spindle_core",
        outer_elem="hub_shell",
        margin=0.0,
        name="spindle_is_centered_inside_hub_shell",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        min_gap=0.09,
        positive_elem="arm_0",
        negative_elem="weather_hood",
        name="arm_plane_clears_weather_hood",
    )

    with ctx.pose({spin: math.tau / 3.0}):
        ctx.expect_contact(
            rotor,
            frame,
            elem_a="hub_shell",
            elem_b="bearing_seat",
            name="hub_shell_remains_supported_when_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
