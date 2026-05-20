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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


OPTICAL_AXIS_RPY = (0.0, math.pi / 2.0, 0.0)


def _shell_from_sections(name: str, sections, *, segments: int = 72):
    outer_profile = [(outer_radius, z) for z, outer_radius, _ in sections]
    inner_profile = [(inner_radius, z) for z, _, inner_radius in sections]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        name,
    )


def _beveled_ring_shell(
    name: str,
    *,
    length: float,
    outer_radius: float,
    inner_radius: float,
    bevel: float = 0.002,
):
    half = length / 2.0
    bevel = min(bevel, length / 3.0)
    return _shell_from_sections(
        name,
        (
            (-half, outer_radius - 0.0015, inner_radius),
            (-half + bevel, outer_radius, inner_radius),
            (half - bevel, outer_radius, inner_radius),
            (half, outer_radius - 0.0015, inner_radius),
        ),
    )


def _add_gear_teeth(
    part,
    *,
    prefix: str,
    axial_length: float,
    shell_radius: float,
    tooth_depth: float,
    tooth_width: float,
    count: int,
    material,
):
    center_radius = shell_radius + tooth_depth * 0.25
    for index in range(count):
        angle = (2.0 * math.pi * index) / count
        part.visual(
            Box((axial_length, tooth_depth, tooth_width)),
            origin=Origin(
                xyz=(
                    0.0,
                    center_radius * math.cos(angle),
                    center_radius * math.sin(angle),
                ),
                rpy=(angle, 0.0, 0.0),
            ),
            material=material,
            name=f"{prefix}_tooth_{index:02d}",
        )


def _add_inner_bearing_pads(
    part,
    *,
    prefix: str,
    axial_length: float,
    shell_inner_radius: float,
    contact_radius: float,
    pad_width: float,
    count: int,
    material,
):
    radial_length = shell_inner_radius + 0.001 - contact_radius
    center_radius = contact_radius + radial_length / 2.0
    for index in range(count):
        angle = (2.0 * math.pi * index) / count
        part.visual(
            Box((axial_length, radial_length, pad_width)),
            origin=Origin(
                xyz=(
                    0.0,
                    center_radius * math.cos(angle),
                    center_radius * math.sin(angle),
                ),
                rpy=(angle, 0.0, 0.0),
            ),
            material=material,
            name=f"{prefix}_pad_{index:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cine_zoom_lens")

    alloy = model.material("alloy", rgba=(0.22, 0.23, 0.25, 1.0))
    control_black = model.material("control_black", rgba=(0.10, 0.10, 0.11, 1.0))
    matte_black = model.material("matte_black", rgba=(0.05, 0.05, 0.06, 1.0))
    mount_steel = model.material("mount_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    coated_glass = model.material("coated_glass", rgba=(0.32, 0.39, 0.48, 0.62))

    barrel_body = model.part("barrel_body")
    barrel_body.visual(
        _shell_from_sections(
            "rear_shoulder_shell",
            (
                (0.000, 0.043, 0.029),
                (0.006, 0.044, 0.029),
                (0.028, 0.044, 0.031),
                (0.032, 0.047, 0.036),
            ),
        ),
        origin=Origin(rpy=OPTICAL_AXIS_RPY),
        material=alloy,
        name="rear_shoulder",
    )
    barrel_body.visual(
        _shell_from_sections(
            "iris_bed_shell",
            (
                (0.030, 0.046, 0.036),
                (0.110, 0.046, 0.036),
            ),
        ),
        origin=Origin(rpy=OPTICAL_AXIS_RPY),
        material=alloy,
        name="iris_bed",
    )
    barrel_body.visual(
        _shell_from_sections(
            "zoom_track_shell",
            (
                (0.110, 0.048, 0.045),
                (0.172, 0.048, 0.045),
            ),
        ),
        origin=Origin(rpy=OPTICAL_AXIS_RPY),
        material=alloy,
        name="zoom_track",
    )
    barrel_body.visual(
        _shell_from_sections(
            "front_guide_shell",
            (
                (0.172, 0.050, 0.045),
                (0.208, 0.050, 0.045),
            ),
        ),
        origin=Origin(rpy=OPTICAL_AXIS_RPY),
        material=alloy,
        name="front_guide",
    )
    barrel_body.visual(
        Box((0.024, 0.001, 0.008)),
        origin=Origin(xyz=(0.190, 0.0455, 0.0)),
        material=alloy,
        name="guide_rail_pos",
    )
    barrel_body.visual(
        Box((0.024, 0.001, 0.008)),
        origin=Origin(xyz=(0.190, -0.0455, 0.0)),
        material=alloy,
        name="guide_rail_neg",
    )
    barrel_body.inertial = Inertial.from_geometry(
        Box((0.210, 0.120, 0.120)),
        mass=1.8,
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
    )

    inner_barrel = model.part("inner_barrel")
    inner_barrel.visual(
        _shell_from_sections(
            "inner_guide_land_shell",
            (
                (-0.040, 0.044, 0.032),
                (0.020, 0.044, 0.032),
            ),
        ),
        origin=Origin(rpy=OPTICAL_AXIS_RPY),
        material=alloy,
        name="guide_land",
    )
    inner_barrel.visual(
        _shell_from_sections(
            "guide_transition_shell",
            (
                (0.018, 0.044, 0.032),
                (0.030, 0.0445, 0.036),
                (0.042, 0.045, 0.039),
            ),
        ),
        origin=Origin(rpy=OPTICAL_AXIS_RPY),
        material=alloy,
        name="guide_transition",
    )
    inner_barrel.visual(
        _shell_from_sections(
            "focus_track_shell",
            (
                (0.040, 0.045, 0.039),
                (0.118, 0.045, 0.039),
            ),
        ),
        origin=Origin(rpy=OPTICAL_AXIS_RPY),
        material=alloy,
        name="focus_track",
    )
    inner_barrel.visual(
        Box((0.020, 0.002, 0.008)),
        origin=Origin(xyz=(0.018, 0.044, 0.0)),
        material=alloy,
        name="guide_pad_pos",
    )
    inner_barrel.visual(
        Box((0.020, 0.002, 0.008)),
        origin=Origin(xyz=(0.018, -0.044, 0.0)),
        material=alloy,
        name="guide_pad_neg",
    )
    inner_barrel.visual(
        _shell_from_sections(
            "front_transition_shell",
            (
                (0.106, 0.046, 0.039),
                (0.118, 0.053, 0.047),
            ),
        ),
        origin=Origin(rpy=OPTICAL_AXIS_RPY),
        material=alloy,
        name="front_transition",
    )
    inner_barrel.visual(
        _shell_from_sections(
            "front_bezel_shell",
            (
                (0.106, 0.053, 0.047),
                (0.150, 0.053, 0.047),
            ),
        ),
        origin=Origin(rpy=OPTICAL_AXIS_RPY),
        material=alloy,
        name="front_bezel",
    )
    inner_barrel.visual(
        _shell_from_sections(
            "front_retainer_shell",
            (
                (0.136, 0.047, 0.0455),
                (0.140, 0.047, 0.0455),
            ),
        ),
        origin=Origin(rpy=OPTICAL_AXIS_RPY),
        material=matte_black,
        name="front_retainer",
    )
    inner_barrel.visual(
        Cylinder(radius=0.046, length=0.010),
        origin=Origin(xyz=(0.141, 0.0, 0.0), rpy=OPTICAL_AXIS_RPY),
        material=coated_glass,
        name="front_element",
    )
    inner_barrel.inertial = Inertial.from_geometry(
        Box((0.200, 0.108, 0.108)),
        mass=0.8,
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
    )

    zoom_ring = model.part("zoom_ring")
    zoom_ring.visual(
        _beveled_ring_shell(
            "zoom_ring_shell",
            length=0.050,
            outer_radius=0.060,
            inner_radius=0.056,
        ),
        origin=Origin(rpy=OPTICAL_AXIS_RPY),
        material=control_black,
        name="zoom_shell",
    )
    _add_gear_teeth(
        zoom_ring,
        prefix="zoom",
        axial_length=0.050,
        shell_radius=0.060,
        tooth_depth=0.0045,
        tooth_width=0.005,
        count=28,
        material=control_black,
    )
    _add_inner_bearing_pads(
        zoom_ring,
        prefix="zoom",
        axial_length=0.012,
        shell_inner_radius=0.056,
        contact_radius=0.048,
        pad_width=0.012,
        count=3,
        material=control_black,
    )
    zoom_ring.inertial = Inertial.from_geometry(
        Box((0.055, 0.130, 0.130)),
        mass=0.18,
        origin=Origin(),
    )

    iris_ring = model.part("iris_ring")
    iris_ring.visual(
        _beveled_ring_shell(
            "iris_ring_shell",
            length=0.022,
            outer_radius=0.057,
            inner_radius=0.053,
            bevel=0.0015,
        ),
        origin=Origin(rpy=OPTICAL_AXIS_RPY),
        material=control_black,
        name="iris_shell",
    )
    _add_gear_teeth(
        iris_ring,
        prefix="iris",
        axial_length=0.022,
        shell_radius=0.057,
        tooth_depth=0.0035,
        tooth_width=0.0045,
        count=22,
        material=control_black,
    )
    _add_inner_bearing_pads(
        iris_ring,
        prefix="iris",
        axial_length=0.010,
        shell_inner_radius=0.053,
        contact_radius=0.046,
        pad_width=0.010,
        count=3,
        material=control_black,
    )
    iris_ring.inertial = Inertial.from_geometry(
        Box((0.026, 0.122, 0.122)),
        mass=0.09,
        origin=Origin(),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        _beveled_ring_shell(
            "focus_ring_shell",
            length=0.038,
            outer_radius=0.059,
            inner_radius=0.050,
        ),
        origin=Origin(rpy=OPTICAL_AXIS_RPY),
        material=control_black,
        name="focus_shell",
    )
    _add_gear_teeth(
        focus_ring,
        prefix="focus",
        axial_length=0.038,
        shell_radius=0.059,
        tooth_depth=0.0045,
        tooth_width=0.005,
        count=26,
        material=control_black,
    )
    _add_inner_bearing_pads(
        focus_ring,
        prefix="focus",
        axial_length=0.010,
        shell_inner_radius=0.050,
        contact_radius=0.045,
        pad_width=0.010,
        count=3,
        material=control_black,
    )
    focus_ring.inertial = Inertial.from_geometry(
        Box((0.042, 0.128, 0.128)),
        mass=0.14,
        origin=Origin(),
    )

    pl_mount = model.part("pl_mount")
    pl_mount.visual(
        _shell_from_sections(
            "pl_mount_flange_shell",
            (
                (-0.018, 0.038, 0.027),
                (0.000, 0.038, 0.027),
            ),
        ),
        origin=Origin(rpy=OPTICAL_AXIS_RPY),
        material=mount_steel,
        name="mount_flange",
    )
    pl_mount.visual(
        _shell_from_sections(
            "pl_mount_spigot_shell",
            (
                (0.000, 0.029, 0.027),
                (0.004, 0.029, 0.027),
            ),
        ),
        origin=Origin(rpy=OPTICAL_AXIS_RPY),
        material=mount_steel,
        name="mount_spigot",
    )
    for index in range(4):
        angle = (2.0 * math.pi * index) / 4.0
        pl_mount.visual(
            Box((0.010, 0.008, 0.014)),
            origin=Origin(
                xyz=(-0.009, 0.040 * math.cos(angle), 0.040 * math.sin(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=mount_steel,
            name=f"lug_{index}",
        )
    pl_mount.inertial = Inertial.from_geometry(
        Box((0.024, 0.090, 0.090)),
        mass=0.22,
        origin=Origin(xyz=(-0.007, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_zoom_ring",
        ArticulationType.REVOLUTE,
        parent=barrel_body,
        child=zoom_ring,
        origin=Origin(xyz=(0.141, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-0.75,
            upper=0.75,
        ),
    )
    model.articulation(
        "body_to_inner_barrel",
        ArticulationType.PRISMATIC,
        parent=barrel_body,
        child=inner_barrel,
        origin=Origin(xyz=(0.172, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.08,
            lower=0.0,
            upper=0.055,
        ),
    )
    model.articulation(
        "body_to_iris_ring",
        ArticulationType.REVOLUTE,
        parent=barrel_body,
        child=iris_ring,
        origin=Origin(xyz=(0.084, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-0.45,
            upper=0.45,
        ),
    )
    model.articulation(
        "inner_barrel_to_focus_ring",
        ArticulationType.REVOLUTE,
        parent=inner_barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.086, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=-1.4,
            upper=1.4,
        ),
    )
    model.articulation(
        "body_to_pl_mount",
        ArticulationType.FIXED,
        parent=barrel_body,
        child=pl_mount,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel_body = object_model.get_part("barrel_body")
    inner_barrel = object_model.get_part("inner_barrel")
    zoom_ring = object_model.get_part("zoom_ring")
    focus_ring = object_model.get_part("focus_ring")
    iris_ring = object_model.get_part("iris_ring")
    pl_mount = object_model.get_part("pl_mount")

    zoom_joint = object_model.get_articulation("body_to_zoom_ring")
    extension_joint = object_model.get_articulation("body_to_inner_barrel")
    iris_joint = object_model.get_articulation("body_to_iris_ring")
    focus_joint = object_model.get_articulation("inner_barrel_to_focus_ring")

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
        "zoom ring spins about the optical axis",
        tuple(zoom_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={zoom_joint.axis}",
    )
    ctx.check(
        "inner barrel extends along the optical axis",
        tuple(extension_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={extension_joint.axis}",
    )
    ctx.check(
        "focus ring spins about the optical axis",
        tuple(focus_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={focus_joint.axis}",
    )
    ctx.check(
        "iris ring spins about the optical axis",
        tuple(iris_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={iris_joint.axis}",
    )

    ctx.expect_contact(
        zoom_ring,
        barrel_body,
        elem_a="zoom_pad_00",
        elem_b="zoom_track",
        name="zoom ring is mounted on the zoom sleeve",
    )
    ctx.expect_contact(
        iris_ring,
        barrel_body,
        elem_a="iris_pad_00",
        elem_b="iris_bed",
        name="iris ring is mounted on the barrel mid-section",
    )
    ctx.expect_contact(
        focus_ring,
        inner_barrel,
        elem_a="focus_pad_00",
        elem_b="focus_track",
        name="focus ring is mounted on the inner barrel nose",
    )
    ctx.expect_contact(
        pl_mount,
        barrel_body,
        elem_a="mount_spigot",
        elem_b="rear_shoulder",
        name="PL mount plate seats into the rear barrel shoulder",
    )
    ctx.expect_contact(
        inner_barrel,
        barrel_body,
        elem_a="guide_pad_pos",
        elem_b="guide_rail_pos",
        name="inner barrel rides on the fixed guide rail",
    )

    ctx.expect_within(
        inner_barrel,
        barrel_body,
        axes="yz",
        inner_elem="guide_land",
        outer_elem="front_guide",
        margin=0.0,
        name="inner barrel guide land stays centered in the front sleeve",
    )
    ctx.expect_overlap(
        inner_barrel,
        barrel_body,
        axes="x",
        elem_a="guide_land",
        elem_b="front_guide",
        min_overlap=0.018,
        name="inner barrel remains inserted in the front guide at rest",
    )
    ctx.expect_origin_gap(
        focus_ring,
        inner_barrel,
        axis="x",
        min_gap=0.075,
        max_gap=0.095,
        name="focus ring sits near the front element group",
    )

    extension_upper = 0.055
    rest_position = ctx.part_world_position(inner_barrel)
    with ctx.pose({extension_joint: extension_upper}):
        ctx.expect_within(
            inner_barrel,
            barrel_body,
            axes="yz",
            inner_elem="guide_land",
            outer_elem="front_guide",
            margin=0.0,
            name="extended inner barrel stays coaxial with the guide sleeve",
        )
        ctx.expect_overlap(
            inner_barrel,
            barrel_body,
            axes="x",
            elem_a="guide_land",
            elem_b="front_guide",
            min_overlap=0.020,
            name="extended inner barrel retains insertion in the guide sleeve",
        )
        extended_position = ctx.part_world_position(inner_barrel)

    ctx.check(
        "zoom extension moves the front barrel forward",
        rest_position is not None
        and extended_position is not None
        and extended_position[0] > rest_position[0] + 0.045,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
