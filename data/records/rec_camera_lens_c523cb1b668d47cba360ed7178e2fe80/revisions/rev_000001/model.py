from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


LATHE_SEGMENTS = 84
FRONT_EXTENSION_TRAVEL = 0.020


def _shell_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
):
    outer_profile = [
        (outer_radius, -0.5 * length),
        (outer_radius, 0.5 * length),
    ]
    inner_profile = [
        (inner_radius, -0.5 * length),
        (inner_radius, 0.5 * length),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=LATHE_SEGMENTS,
        ),
        name,
    )


def _add_ribbed_ring(
    part,
    *,
    mesh_prefix: str,
    inner_radius: float,
    base_outer_radius: float,
    ridge_outer_radius: float,
    length: float,
    ridge_length: float,
    ridge_offsets: tuple[float, ...],
    material,
    base_name: str,
) -> None:
    base_mesh = _shell_mesh(
        f"{mesh_prefix}_base",
        outer_radius=base_outer_radius,
        inner_radius=inner_radius,
        length=length,
    )
    ridge_mesh = _shell_mesh(
        f"{mesh_prefix}_ridge",
        outer_radius=ridge_outer_radius,
        inner_radius=inner_radius,
        length=ridge_length,
    )

    part.visual(base_mesh, material=material, name=base_name)
    for index, z_offset in enumerate(ridge_offsets):
        part.visual(
            ridge_mesh,
            origin=Origin(xyz=(0.0, 0.0, z_offset)),
            material=material,
            name=f"{base_name}_ridge_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_kit_zoom_lens")

    body_black = model.material("body_black", rgba=(0.11, 0.11, 0.12, 1.0))
    satin_black = model.material("satin_black", rgba=(0.17, 0.17, 0.18, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))
    mount_silver = model.material("mount_silver", rgba=(0.70, 0.72, 0.74, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.14, 0.18, 0.22, 0.48))

    mount_shell = _shell_mesh(
        "lens_mount_shell",
        outer_radius=0.0314,
        inner_radius=0.0240,
        length=0.0038,
    )
    rear_core = _shell_mesh(
        "rear_core_shell",
        outer_radius=0.02855,
        inner_radius=0.0258,
        length=0.0072,
    )
    center_shell = _shell_mesh(
        "center_shell",
        outer_radius=0.0300,
        inner_radius=0.0258,
        length=0.0110,
    )
    zoom_support = _shell_mesh(
        "zoom_support_shell",
        outer_radius=0.02880,
        inner_radius=0.0256,
        length=0.0090,
    )
    front_lip = _shell_mesh(
        "front_lip_shell",
        outer_radius=0.0295,
        inner_radius=0.0254,
        length=0.0050,
    )

    outer_barrel = model.part("outer_barrel")
    outer_barrel.visual(
        mount_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.0019)),
        material=mount_silver,
        name="mount_shell",
    )
    outer_barrel.visual(
        rear_core,
        origin=Origin(xyz=(0.0, 0.0, 0.0074)),
        material=body_black,
        name="unlock_support",
    )
    outer_barrel.visual(
        center_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.0165)),
        material=body_black,
        name="center_shell",
    )
    outer_barrel.visual(
        zoom_support,
        origin=Origin(xyz=(0.0, 0.0, 0.0265)),
        material=body_black,
        name="zoom_support",
    )
    outer_barrel.visual(
        front_lip,
        origin=Origin(xyz=(0.0, 0.0, 0.0335)),
        material=satin_black,
        name="guide_shell",
    )

    unlock_ring = model.part("unlock_ring")
    _add_ribbed_ring(
        unlock_ring,
        mesh_prefix="unlock_ring",
        inner_radius=0.02895,
        base_outer_radius=0.0310,
        ridge_outer_radius=0.0317,
        length=0.0072,
        ridge_length=0.0010,
        ridge_offsets=(-0.0020, 0.0, 0.0020),
        material=rubber_black,
        base_name="unlock_shell",
    )

    zoom_ring = model.part("zoom_ring")
    _add_ribbed_ring(
        zoom_ring,
        mesh_prefix="zoom_ring",
        inner_radius=0.02910,
        base_outer_radius=0.0314,
        ridge_outer_radius=0.0324,
        length=0.0090,
        ridge_length=0.0011,
        ridge_offsets=(-0.0030, -0.0015, 0.0, 0.0015, 0.0030),
        material=rubber_black,
        base_name="zoom_shell",
    )

    front_guide = _shell_mesh(
        "front_guide_shell",
        outer_radius=0.02485,
        inner_radius=0.0226,
        length=0.0360,
    )
    front_grip = _shell_mesh(
        "front_grip_shell",
        outer_radius=0.0264,
        inner_radius=0.0226,
        length=0.0080,
    )
    front_rim = _shell_mesh(
        "front_rim_shell",
        outer_radius=0.0278,
        inner_radius=0.0212,
        length=0.0050,
    )
    front_grip_ridge = _shell_mesh(
        "front_grip_ridge",
        outer_radius=0.0272,
        inner_radius=0.0226,
        length=0.0012,
    )

    front_barrel = model.part("front_barrel")
    front_barrel.visual(
        front_guide,
        origin=Origin(xyz=(0.0, 0.0, -0.0060)),
        material=satin_black,
        name="guide_section",
    )
    front_barrel.visual(
        front_grip,
        origin=Origin(xyz=(0.0, 0.0, 0.0160)),
        material=satin_black,
        name="front_grip",
    )
    for index, z_offset in enumerate((0.0135, 0.0160, 0.0185)):
        front_barrel.visual(
            front_grip_ridge,
            origin=Origin(xyz=(0.0, 0.0, z_offset)),
            material=body_black,
            name=f"front_grip_ridge_{index}",
        )
    front_barrel.visual(
        front_rim,
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=body_black,
        name="front_rim",
    )
    front_barrel.visual(
        Cylinder(radius=0.0212, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0206)),
        material=glass_dark,
        name="front_element",
    )

    model.articulation(
        "unlock_spin",
        ArticulationType.CONTINUOUS,
        parent=outer_barrel,
        child=unlock_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0074)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )
    model.articulation(
        "zoom_spin",
        ArticulationType.CONTINUOUS,
        parent=outer_barrel,
        child=zoom_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0265)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.7, velocity=7.0),
    )
    model.articulation(
        "front_extension",
        ArticulationType.PRISMATIC,
        parent=outer_barrel,
        child=front_barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.0240)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=0.18,
            lower=0.0,
            upper=FRONT_EXTENSION_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer_barrel = object_model.get_part("outer_barrel")
    unlock_ring = object_model.get_part("unlock_ring")
    zoom_ring = object_model.get_part("zoom_ring")
    front_barrel = object_model.get_part("front_barrel")

    unlock_spin = object_model.get_articulation("unlock_spin")
    zoom_spin = object_model.get_articulation("zoom_spin")
    front_extension = object_model.get_articulation("front_extension")

    ctx.expect_origin_distance(
        unlock_ring,
        outer_barrel,
        axes="xy",
        max_dist=0.0005,
        name="unlock ring stays coaxial with the barrel",
    )
    ctx.expect_origin_distance(
        zoom_ring,
        outer_barrel,
        axes="xy",
        max_dist=0.0005,
        name="zoom ring stays coaxial with the barrel",
    )
    ctx.expect_origin_distance(
        front_barrel,
        outer_barrel,
        axes="xy",
        max_dist=0.0005,
        name="front barrel stays centered on the optical axis",
    )

    with ctx.pose({front_extension: 0.0}):
        ctx.expect_within(
            front_barrel,
            outer_barrel,
            axes="xy",
            margin=0.0,
            name="collapsed front barrel fits within the outer barrel diameter",
        )
        ctx.expect_overlap(
            front_barrel,
            outer_barrel,
            axes="z",
            min_overlap=0.030,
            name="collapsed front barrel remains substantially nested",
        )
        rest_position = ctx.part_world_position(front_barrel)

    with ctx.pose(
        {
            unlock_spin: 1.4,
            zoom_spin: -1.1,
            front_extension: FRONT_EXTENSION_TRAVEL,
        }
    ):
        ctx.expect_origin_distance(
            unlock_ring,
            outer_barrel,
            axes="xy",
            max_dist=0.0005,
            name="unlock ring remains centered while rotated",
        )
        ctx.expect_origin_distance(
            zoom_ring,
            outer_barrel,
            axes="xy",
            max_dist=0.0005,
            name="zoom ring remains centered while rotated",
        )
        ctx.expect_within(
            front_barrel,
            outer_barrel,
            axes="xy",
            margin=0.0,
            name="extended front barrel stays coaxial",
        )
        ctx.expect_overlap(
            front_barrel,
            outer_barrel,
            axes="z",
            min_overlap=0.015,
            name="extended front barrel retains insertion",
        )
        extended_position = ctx.part_world_position(front_barrel)

    ctx.check(
        "front barrel extends forward along the optical axis",
        rest_position is not None
        and extended_position is not None
        and extended_position[2] > rest_position[2] + 0.015,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    return ctx.report()


object_model = build_object_model()
