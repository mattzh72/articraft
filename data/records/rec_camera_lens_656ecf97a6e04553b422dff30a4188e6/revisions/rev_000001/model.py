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


def _shell_mesh(
    name: str,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 72,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _add_box_ring_features(
    part,
    *,
    base_name: str,
    count: int,
    radius: float,
    z: float,
    radial_size: float,
    tangential_size: float,
    axial_size: float,
    material,
    start_angle: float = 0.0,
) -> None:
    for index in range(count):
        angle = start_angle + (2.0 * math.pi * index) / count
        part.visual(
            Box((radial_size, tangential_size, axial_size)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), z),
                rpy=(0.0, 0.0, angle),
            ),
            material=material,
            name=f"{base_name}_{index:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="teleconverter_2x")

    body_black = model.material("body_black", rgba=(0.09, 0.09, 0.10, 1.0))
    grip_black = model.material("grip_black", rgba=(0.14, 0.14, 0.15, 1.0))
    mount_metal = model.material("mount_metal", rgba=(0.72, 0.73, 0.75, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.30, 0.31, 0.33, 1.0))
    accent_red = model.material("accent_red", rgba=(0.74, 0.10, 0.12, 1.0))
    label_gray = model.material("label_gray", rgba=(0.36, 0.37, 0.39, 1.0))

    barrel_shell = _shell_mesh(
        "teleconverter_barrel_shell",
        [
            (0.0336, -0.0130),
            (0.0336, 0.0140),
        ],
        [
            (0.0221, -0.0130),
            (0.0214, 0.0000),
            (0.0218, 0.0140),
        ],
    )
    rear_mount_core = _shell_mesh(
        "teleconverter_rear_mount_core",
        [
            (0.0280, -0.0260),
            (0.0280, -0.0140),
        ],
        [
            (0.0228, -0.0260),
            (0.0224, -0.0140),
        ],
    )
    alignment_mount_core = _shell_mesh(
        "teleconverter_alignment_mount_core",
        [
            (0.0286, -0.0120),
            (0.0286, -0.0050),
        ],
        [
            (0.0224, -0.0120),
            (0.0220, -0.0050),
        ],
    )
    front_mount_core = _shell_mesh(
        "teleconverter_front_mount_core",
        [
            (0.0280, 0.0140),
            (0.0280, 0.0260),
        ],
        [
            (0.0220, 0.0140),
            (0.0226, 0.0260),
        ],
    )
    rear_retaining_shoulder = _shell_mesh(
        "teleconverter_rear_retaining_shoulder",
        [
            (0.0340, -0.0140),
            (0.0340, -0.0130),
        ],
        [
            (0.0224, -0.0140),
            (0.0220, -0.0130),
        ],
    )
    alignment_retaining_shoulder = _shell_mesh(
        "teleconverter_alignment_retaining_shoulder",
        [
            (0.0315, -0.0050),
            (0.0315, -0.0040),
        ],
        [
            (0.0220, -0.0050),
            (0.0218, -0.0040),
        ],
    )
    front_retaining_shoulder = _shell_mesh(
        "teleconverter_front_retaining_shoulder",
        [
            (0.0340, 0.0130),
            (0.0340, 0.0140),
        ],
        [
            (0.0218, 0.0130),
            (0.0220, 0.0140),
        ],
    )
    grip_band = _shell_mesh(
        "teleconverter_grip_band",
        [
            (0.0340, -0.0015),
            (0.0346, -0.0010),
            (0.0346, 0.0090),
            (0.0340, 0.0095),
        ],
        [
            (0.0330, -0.0015),
            (0.0330, 0.0095),
        ],
    )
    front_ring_shell = _shell_mesh(
        "front_bayonet_ring_shell",
        [
            (0.0343, -0.0060),
            (0.0355, -0.0040),
            (0.0355, 0.0040),
            (0.0341, 0.0060),
        ],
        [
            (0.0300, -0.0060),
            (0.0300, 0.0060),
        ],
    )
    rear_ring_shell = _shell_mesh(
        "rear_bayonet_ring_shell",
        [
            (0.0340, -0.0060),
            (0.0349, -0.0040),
            (0.0349, 0.0040),
            (0.0338, 0.0060),
        ],
        [
            (0.0298, -0.0060),
            (0.0298, 0.0060),
        ],
    )
    alignment_ring_shell = _shell_mesh(
        "alignment_ring_shell_mesh",
        [
            (0.0320, -0.0035),
            (0.0328, -0.0022),
            (0.0328, 0.0022),
            (0.0320, 0.0035),
        ],
        [
            (0.0305, -0.0035),
            (0.0305, 0.0035),
        ],
    )

    body = model.part("body")
    body.visual(barrel_shell, material=body_black, name="barrel_shell")
    body.visual(rear_mount_core, material=body_black, name="rear_mount_core")
    body.visual(alignment_mount_core, material=body_black, name="alignment_mount_core")
    body.visual(front_mount_core, material=body_black, name="front_mount_core")
    body.visual(rear_retaining_shoulder, material=dark_metal, name="rear_retaining_shoulder")
    body.visual(
        alignment_retaining_shoulder,
        material=dark_metal,
        name="alignment_retaining_shoulder",
    )
    body.visual(front_retaining_shoulder, material=dark_metal, name="front_retaining_shoulder")
    body.visual(
        Box((0.0060, 0.0056, 0.0100)),
        origin=Origin(xyz=(0.0000, -0.0307, -0.0075)),
        material=dark_metal,
        name="alignment_drive_housing",
    )
    body.visual(grip_band, material=grip_black, name="center_grip_band")
    body.visual(
        Box((0.0110, 0.0016, 0.0060)),
        origin=Origin(xyz=(0.0000, -0.0332, 0.0040)),
        material=label_gray,
        name="spec_plate",
    )
    body.visual(
        Cylinder(radius=0.0014, length=0.0012),
        origin=Origin(xyz=(0.0000, 0.0338, 0.0100), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent_red,
        name="mount_index_dot",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0340, length=0.0520),
        mass=0.22,
        origin=Origin(),
    )

    front_bayonet_ring = model.part("front_bayonet_ring")
    front_bayonet_ring.visual(front_ring_shell, material=mount_metal, name="front_rotor_shell")
    for name, angle_deg, tangential in (
        ("clocking_tab", 12.0, 0.0105),
        ("bayonet_tab_1", 158.0, 0.0080),
        ("bayonet_tab_2", 286.0, 0.0090),
    ):
        angle = math.radians(angle_deg)
        front_bayonet_ring.visual(
            Box((0.0052, tangential, 0.0032)),
            origin=Origin(
                xyz=(0.0330 * math.cos(angle), 0.0330 * math.sin(angle), 0.0052),
                rpy=(0.0, 0.0, angle),
            ),
            material=mount_metal,
            name=name,
        )
    _add_box_ring_features(
        front_bayonet_ring,
        base_name="grip_rib",
        count=12,
        radius=0.0350,
        z=0.0000,
        radial_size=0.0016,
        tangential_size=0.0040,
        axial_size=0.0092,
        material=dark_metal,
        start_angle=math.radians(7.5),
    )
    front_bayonet_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0358, length=0.0120),
        mass=0.07,
        origin=Origin(),
    )

    rear_bayonet_ring = model.part("rear_bayonet_ring")
    rear_bayonet_ring.visual(rear_ring_shell, material=mount_metal, name="rear_mount_shell")
    for index, (angle_deg, tangential) in enumerate(((32.0, 0.0090), (176.0, 0.0082), (302.0, 0.0100))):
        angle = math.radians(angle_deg)
        rear_bayonet_ring.visual(
            Box((0.0050, tangential, 0.0030)),
            origin=Origin(
                xyz=(0.0327 * math.cos(angle), 0.0327 * math.sin(angle), -0.0050),
                rpy=(0.0, 0.0, angle),
            ),
            material=mount_metal,
            name=f"rear_bayonet_tab_{index}",
        )
    rear_bayonet_ring.visual(
        Box((0.0080, 0.0026, 0.0048)),
        origin=Origin(xyz=(0.0000, -0.0337, -0.0010)),
        material=dark_metal,
        name="aperture_lever_bridge",
    )
    rear_bayonet_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0350, length=0.0120),
        mass=0.07,
        origin=Origin(),
    )

    alignment_peg_ring = model.part("alignment_peg_ring")
    alignment_peg_ring.visual(alignment_ring_shell, material=dark_metal, name="alignment_ring_shell")
    _add_box_ring_features(
        alignment_peg_ring,
        base_name="alignment_flute",
        count=6,
        radius=0.0322,
        z=0.0000,
        radial_size=0.0014,
        tangential_size=0.0032,
        axial_size=0.0050,
        material=grip_black,
        start_angle=math.radians(15.0),
    )
    for name, angle_deg, radius, peg_radius, peg_length in (
        ("index_peg", 4.0, 0.0310, 0.0016, 0.0040),
        ("alignment_peg_1", 128.0, 0.0308, 0.0012, 0.0036),
        ("alignment_peg_2", 252.0, 0.0309, 0.0012, 0.0036),
    ):
        angle = math.radians(angle_deg)
        alignment_peg_ring.visual(
            Cylinder(radius=peg_radius, length=peg_length),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), -0.0024),
            ),
            material=mount_metal,
            name=name,
        )
    alignment_peg_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0330, length=0.0070),
        mass=0.03,
        origin=Origin(),
    )

    model.articulation(
        "front_mount_rotation",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_bayonet_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0200)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=4.0,
            lower=-0.75,
            upper=0.75,
        ),
    )
    model.articulation(
        "rear_mount_attachment",
        ArticulationType.FIXED,
        parent=body,
        child=rear_bayonet_ring,
        origin=Origin(xyz=(0.0, 0.0, -0.0200)),
    )
    model.articulation(
        "alignment_ring_rotation",
        ArticulationType.REVOLUTE,
        parent=body,
        child=alignment_peg_ring,
        origin=Origin(xyz=(0.0, 0.0, -0.0085)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=6.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    front_bayonet_ring = object_model.get_part("front_bayonet_ring")
    rear_bayonet_ring = object_model.get_part("rear_bayonet_ring")
    alignment_peg_ring = object_model.get_part("alignment_peg_ring")

    front_mount_rotation = object_model.get_articulation("front_mount_rotation")
    rear_mount_attachment = object_model.get_articulation("rear_mount_attachment")
    alignment_ring_rotation = object_model.get_articulation("alignment_ring_rotation")

    ctx.check(
        "teleconverter uses expected parts",
        {
            body.name,
            front_bayonet_ring.name,
            rear_bayonet_ring.name,
            alignment_peg_ring.name,
        }
        == {"body", "front_bayonet_ring", "rear_bayonet_ring", "alignment_peg_ring"},
        details="One or more required teleconverter parts are missing.",
    )
    ctx.check(
        "front and alignment rings rotate about optical axis",
        front_mount_rotation.axis == (0.0, 0.0, 1.0)
        and alignment_ring_rotation.axis == (0.0, 0.0, 1.0),
        details=(
            f"front axis={front_mount_rotation.axis}, "
            f"alignment axis={alignment_ring_rotation.axis}"
        ),
    )
    ctx.check(
        "rear mount remains fixed",
        rear_mount_attachment.articulation_type == ArticulationType.FIXED,
        details=f"rear articulation type={rear_mount_attachment.articulation_type}",
    )

    ctx.expect_origin_distance(
        front_bayonet_ring,
        body,
        axes="xy",
        max_dist=0.0005,
        name="front ring is concentric with barrel",
    )
    ctx.expect_origin_distance(
        rear_bayonet_ring,
        body,
        axes="xy",
        max_dist=0.0005,
        name="rear ring is concentric with barrel",
    )
    ctx.expect_origin_distance(
        alignment_peg_ring,
        body,
        axes="xy",
        max_dist=0.0005,
        name="alignment ring is concentric with barrel",
    )
    ctx.expect_origin_gap(
        front_bayonet_ring,
        body,
        axis="z",
        min_gap=0.019,
        max_gap=0.021,
        name="front ring sits on the forward interface",
    )
    ctx.expect_origin_gap(
        body,
        rear_bayonet_ring,
        axis="z",
        min_gap=0.019,
        max_gap=0.021,
        name="rear ring sits on the rear interface",
    )
    ctx.expect_origin_gap(
        body,
        alignment_peg_ring,
        axis="z",
        min_gap=0.0075,
        max_gap=0.0095,
        name="alignment ring sits just ahead of the rear mount",
    )
    ctx.expect_overlap(
        front_bayonet_ring,
        body,
        axes="xy",
        elem_a="front_rotor_shell",
        elem_b="barrel_shell",
        min_overlap=0.060,
        name="front rotor shell overlaps barrel footprint",
    )
    ctx.expect_overlap(
        rear_bayonet_ring,
        body,
        axes="xy",
        elem_a="rear_mount_shell",
        elem_b="barrel_shell",
        min_overlap=0.060,
        name="rear mount shell overlaps barrel footprint",
    )
    ctx.expect_overlap(
        alignment_peg_ring,
        body,
        axes="xy",
        elem_a="alignment_ring_shell",
        elem_b="barrel_shell",
        min_overlap=0.055,
        name="alignment ring overlaps barrel footprint",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    front_tab_rest = _aabb_center(ctx.part_element_world_aabb(front_bayonet_ring, elem="clocking_tab"))
    with ctx.pose({front_mount_rotation: 0.55}):
        ctx.expect_origin_distance(
            front_bayonet_ring,
            body,
            axes="xy",
            max_dist=0.0005,
            name="front ring stays centered while rotated",
        )
        front_tab_rotated = _aabb_center(
            ctx.part_element_world_aabb(front_bayonet_ring, elem="clocking_tab")
        )
    ctx.check(
        "front bayonet ring visibly rotates",
        front_tab_rest is not None
        and front_tab_rotated is not None
        and math.dist(front_tab_rest[:2], front_tab_rotated[:2]) > 0.010
        and abs(front_tab_rest[2] - front_tab_rotated[2]) < 0.0005,
        details=f"rest={front_tab_rest}, rotated={front_tab_rotated}",
    )

    index_peg_rest = _aabb_center(ctx.part_element_world_aabb(alignment_peg_ring, elem="index_peg"))
    with ctx.pose({alignment_ring_rotation: 1.0}):
        ctx.expect_origin_distance(
            alignment_peg_ring,
            body,
            axes="xy",
            max_dist=0.0005,
            name="alignment ring stays centered while rotated",
        )
        index_peg_rotated = _aabb_center(
            ctx.part_element_world_aabb(alignment_peg_ring, elem="index_peg")
        )
    ctx.check(
        "alignment peg ring visibly rotates",
        index_peg_rest is not None
        and index_peg_rotated is not None
        and math.dist(index_peg_rest[:2], index_peg_rotated[:2]) > 0.015
        and abs(index_peg_rest[2] - index_peg_rotated[2]) < 0.0005,
        details=f"rest={index_peg_rest}, rotated={index_peg_rotated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
