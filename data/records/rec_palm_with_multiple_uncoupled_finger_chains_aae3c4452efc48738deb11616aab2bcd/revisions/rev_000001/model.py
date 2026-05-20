from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, pi, sin, tau

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FINGER_ANGLES = (0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)

HUB_RADIUS = 0.040
HUB_THICKNESS = 0.026
HUB_CAP_RADIUS = 0.028
HUB_CAP_THICKNESS = 0.010
ROOT_BRACKET_LENGTH = 0.016
ROOT_BRACKET_WIDTH = 0.021
ROOT_BRACKET_HEIGHT = 0.0045
ROOT_BRACKET_EMBED = 0.0025
ROOT_RADIUS = HUB_RADIUS + ROOT_BRACKET_LENGTH - ROOT_BRACKET_EMBED

PROXIMAL_LENGTH = 0.052
MIDDLE_LENGTH = 0.044
DISTAL_LENGTH = 0.036

PROXIMAL_WIDTH = 0.018
MIDDLE_WIDTH = 0.016
DISTAL_WIDTH = 0.014

PROXIMAL_HEIGHT = 0.010
MIDDLE_HEIGHT = 0.009
DISTAL_HEIGHT = 0.008

JOINT_STACK_HEIGHT = 0.004


def _radial_origin(radius: float, theta: float, z: float = 0.0) -> Origin:
    return Origin(
        xyz=(radius * cos(theta), radius * sin(theta), z),
        rpy=(0.0, 0.0, theta),
    )


def _add_link_geometry(
    part,
    *,
    prefix: str,
    length: float,
    width: float,
    body_height: float,
    shell_material,
    accent_material,
    add_plate: bool,
    add_tip_pad: bool,
) -> None:
    base_block_length = length * 0.22
    tip_block_length = length * 0.18
    beam_start = base_block_length * 0.72
    beam_end = length - tip_block_length * 0.82
    beam_length = max(beam_end - beam_start, length * 0.35)
    base_width = width * 0.84
    tip_width = width * 0.74

    part.visual(
        Box((base_block_length, base_width, JOINT_STACK_HEIGHT)),
        origin=Origin(xyz=(base_block_length / 2.0, 0.0, JOINT_STACK_HEIGHT / 2.0)),
        material=shell_material,
        name=f"{prefix}_base_block",
    )
    part.visual(
        Box((beam_length, width, body_height)),
        origin=Origin(
            xyz=(beam_start + beam_length / 2.0, 0.0, JOINT_STACK_HEIGHT + body_height / 2.0)
        ),
        material=shell_material,
        name=f"{prefix}_beam",
    )
    part.visual(
        Box((tip_block_length, tip_width, JOINT_STACK_HEIGHT)),
        origin=Origin(xyz=(length - tip_block_length / 2.0, 0.0, JOINT_STACK_HEIGHT / 2.0)),
        material=shell_material,
        name=f"{prefix}_tip_block",
    )

    if add_plate:
        plate_length = beam_length * 0.62
        part.visual(
            Box((plate_length, width * 0.72, body_height * 0.24)),
            origin=Origin(
                xyz=(beam_start + beam_length * 0.56, 0.0, JOINT_STACK_HEIGHT + body_height * 1.02)
            ),
            material=accent_material,
            name=f"{prefix}_dorsal_plate",
        )

    if add_tip_pad:
        part.visual(
            Box((tip_block_length * 0.86, width * 0.64, body_height * 0.28)),
            origin=Origin(
                xyz=(length - tip_block_length * 0.54, 0.0, JOINT_STACK_HEIGHT + body_height * 1.01)
            ),
            material=accent_material,
            name=f"{prefix}_tip_pad",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tri_radial_robotic_palm")

    hub_material = model.material("hub_anodized", rgba=(0.18, 0.20, 0.23, 1.0))
    link_material = model.material("link_alloy", rgba=(0.72, 0.74, 0.78, 1.0))
    accent_material = model.material("sensor_blue", rgba=(0.19, 0.46, 0.84, 1.0))

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_THICKNESS),
        material=hub_material,
        name="hub_core",
    )
    hub.visual(
        Cylinder(radius=HUB_CAP_RADIUS, length=HUB_CAP_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, HUB_THICKNESS * 0.22)),
        material=hub_material,
        name="hub_top_cap",
    )
    hub.visual(
        Cylinder(radius=HUB_CAP_RADIUS * 0.92, length=HUB_CAP_THICKNESS * 0.60),
        origin=Origin(xyz=(0.0, 0.0, -HUB_THICKNESS * 0.24)),
        material=hub_material,
        name="hub_bottom_plate",
    )
    hub.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, HUB_THICKNESS * 0.44)),
        material=accent_material,
        name="hub_sensor_dome",
    )

    for index, theta in enumerate(FINGER_ANGLES, start=1):
        hub.visual(
            Box((ROOT_BRACKET_LENGTH, ROOT_BRACKET_WIDTH, ROOT_BRACKET_HEIGHT)),
            origin=_radial_origin(
                HUB_RADIUS + ROOT_BRACKET_LENGTH / 2.0 - ROOT_BRACKET_EMBED,
                theta,
                HUB_THICKNESS / 2.0 + ROOT_BRACKET_HEIGHT / 2.0,
            ),
            material=hub_material,
            name=f"root_bracket_{index}",
        )

        proximal = model.part(f"finger{index}_proximal")
        middle = model.part(f"finger{index}_middle")
        distal = model.part(f"finger{index}_distal")

        _add_link_geometry(
            proximal,
            prefix=f"finger{index}_proximal",
            length=PROXIMAL_LENGTH,
            width=PROXIMAL_WIDTH,
            body_height=PROXIMAL_HEIGHT,
            shell_material=link_material,
            accent_material=accent_material,
            add_plate=True,
            add_tip_pad=False,
        )
        _add_link_geometry(
            middle,
            prefix=f"finger{index}_middle",
            length=MIDDLE_LENGTH,
            width=MIDDLE_WIDTH,
            body_height=MIDDLE_HEIGHT,
            shell_material=link_material,
            accent_material=accent_material,
            add_plate=True,
            add_tip_pad=False,
        )
        _add_link_geometry(
            distal,
            prefix=f"finger{index}_distal",
            length=DISTAL_LENGTH,
            width=DISTAL_WIDTH,
            body_height=DISTAL_HEIGHT,
            shell_material=link_material,
            accent_material=accent_material,
            add_plate=False,
            add_tip_pad=True,
        )

        model.articulation(
            f"finger{index}_root",
            ArticulationType.REVOLUTE,
            parent=hub,
            child=proximal,
            origin=_radial_origin(ROOT_RADIUS, theta, HUB_THICKNESS / 2.0 + ROOT_BRACKET_HEIGHT),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=2.4,
                lower=0.0,
                upper=1.10,
            ),
        )
        model.articulation(
            f"finger{index}_middle_joint",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, JOINT_STACK_HEIGHT)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=9.0,
                velocity=3.0,
                lower=0.0,
                upper=1.25,
            ),
        )
        model.articulation(
            f"finger{index}_distal_joint",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(MIDDLE_LENGTH, 0.0, JOINT_STACK_HEIGHT)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=7.0,
                velocity=3.4,
                lower=0.0,
                upper=1.20,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    expected_parts = ["hub"] + [
        f"finger{finger_index}_{segment}"
        for finger_index in range(1, 4)
        for segment in ("proximal", "middle", "distal")
    ]
    for part_name in expected_parts:
        try:
            object_model.get_part(part_name)
            ctx.check(f"has_{part_name}", True)
        except Exception as exc:  # pragma: no cover - defensive authored check
            ctx.check(f"has_{part_name}", False, str(exc))

    hub = object_model.get_part("hub")
    proximal_parts = []
    expected_pairs: list[tuple[str, str]] = []

    for finger_index in range(1, 4):
        proximal = object_model.get_part(f"finger{finger_index}_proximal")
        middle = object_model.get_part(f"finger{finger_index}_middle")
        distal = object_model.get_part(f"finger{finger_index}_distal")
        root_joint = object_model.get_articulation(f"finger{finger_index}_root")
        middle_joint = object_model.get_articulation(f"finger{finger_index}_middle_joint")
        distal_joint = object_model.get_articulation(f"finger{finger_index}_distal_joint")

        proximal_parts.append(proximal)
        expected_pairs.extend(
            [
                (root_joint.parent, root_joint.child),
                (middle_joint.parent, middle_joint.child),
                (distal_joint.parent, distal_joint.child),
            ]
        )

        for joint in (root_joint, middle_joint, distal_joint):
            ctx.check(
                f"{joint.name}_axis_is_transverse",
                tuple(joint.axis) == (0.0, 1.0, 0.0),
                f"{joint.name} axis is {joint.axis}, expected local +Y transverse hinge axis",
            )

        ctx.expect_contact(
            proximal,
            hub,
            contact_tol=1e-4,
            name=f"finger{finger_index}_root_mount_contact",
        )
        ctx.expect_contact(
            proximal,
            middle,
            contact_tol=1e-4,
            name=f"finger{finger_index}_proximal_middle_contact",
        )
        ctx.expect_contact(
            middle,
            distal,
            contact_tol=1e-4,
            name=f"finger{finger_index}_middle_distal_contact",
        )

        ctx.expect_origin_distance(
            proximal,
            hub,
            axes="xy",
            min_dist=ROOT_RADIUS - 0.001,
            max_dist=ROOT_RADIUS + 0.001,
            name=f"finger{finger_index}_root_radius",
        )
        ctx.expect_origin_distance(
            middle,
            proximal,
            axes="xy",
            min_dist=PROXIMAL_LENGTH - 0.001,
            max_dist=PROXIMAL_LENGTH + 0.001,
            name=f"finger{finger_index}_proximal_length_layout",
        )
        ctx.expect_origin_distance(
            distal,
            middle,
            axes="xy",
            min_dist=MIDDLE_LENGTH - 0.001,
            max_dist=MIDDLE_LENGTH + 0.001,
            name=f"finger{finger_index}_middle_length_layout",
        )

    sorted_pairs = sorted(expected_pairs)
    ctx.check(
        "three_uncoupled_serial_finger_chains",
        sorted_pairs
        == sorted(
            [
                ("hub", "finger1_proximal"),
                ("finger1_proximal", "finger1_middle"),
                ("finger1_middle", "finger1_distal"),
                ("hub", "finger2_proximal"),
                ("finger2_proximal", "finger2_middle"),
                ("finger2_middle", "finger2_distal"),
                ("hub", "finger3_proximal"),
                ("finger3_proximal", "finger3_middle"),
                ("finger3_middle", "finger3_distal"),
            ]
        ),
        f"unexpected articulation topology: {sorted_pairs}",
    )

    proximal_positions = [ctx.part_world_position(part) for part in proximal_parts]
    spacing_ok = all(position is not None for position in proximal_positions)
    spacing_details = ""
    if spacing_ok:
        angles = sorted(
            atan2(position[1], position[0]) % tau
            for position in proximal_positions
            if position is not None
        )
        deltas = [
            (angles[(index + 1) % len(angles)] - angles[index]) % tau
            for index in range(len(angles))
        ]
        spacing_ok = all(abs(delta - tau / 3.0) <= 0.04 for delta in deltas)
        spacing_details = f"angles={angles}, deltas={deltas}"
    else:
        spacing_details = f"proximal_positions={proximal_positions}"
    ctx.check("proximal_origins_evenly_spaced_radially", spacing_ok, spacing_details)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
