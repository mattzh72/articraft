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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    def save_mesh(name: str, geometry: MeshGeometry):
        return mesh_from_geometry(geometry, name)

    def cylindrical_shell(
        *,
        outer_radius: float,
        inner_radius: float,
        z0: float,
        z1: float,
        segments: int = 72,
    ) -> MeshGeometry:
        return LatheGeometry.from_shell_profiles(
            [(outer_radius, z0), (outer_radius, z1)],
            [(inner_radius, z0), (inner_radius, z1)],
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        )

    model = ArticulatedObject(name="cinema_prime_35mm_t1_5")

    anodized_black = model.material("anodized_black", rgba=(0.08, 0.08, 0.09, 1.0))
    satin_black = model.material("satin_black", rgba=(0.12, 0.12, 0.13, 1.0))
    engraved_white = model.material("engraved_white", rgba=(0.94, 0.95, 0.96, 1.0))
    mount_steel = model.material("mount_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    witness_orange = model.material("witness_orange", rgba=(0.93, 0.42, 0.08, 1.0))
    coated_glass = model.material("coated_glass", rgba=(0.11, 0.16, 0.20, 0.88))

    lens_body = model.part("lens_body")

    body_shell = LatheGeometry.from_shell_profiles(
        [
            (0.026, 0.000),
            (0.030, 0.003),
            (0.038, 0.006),
            (0.040, 0.016),
            (0.049, 0.022),
            (0.049, 0.050),
            (0.0465, 0.054),
            (0.0465, 0.086),
            (0.0485, 0.090),
            (0.0485, 0.126),
            (0.0520, 0.138),
            (0.0550, 0.151),
            (0.0570, 0.160),
        ],
        [
            (0.021, 0.000),
            (0.024, 0.010),
            (0.027, 0.020),
            (0.030, 0.050),
            (0.031, 0.090),
            (0.033, 0.126),
            (0.036, 0.148),
            (0.038, 0.160),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
    )
    lens_body.visual(
        save_mesh("lens_body_shell", body_shell),
        material=anodized_black,
        name="lens_body_shell",
    )
    lens_body.visual(
        Cylinder(radius=0.0368, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=coated_glass,
        name="front_element",
    )
    lens_body.visual(
        Box((0.0015, 0.0030, 0.011)),
        origin=Origin(xyz=(0.0, 0.0478, 0.070)),
        material=witness_orange,
        name="focus_index_line",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        radius = 0.026
        lens_body.visual(
            Box((0.012, 0.007, 0.003)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.0015),
                rpy=(0.0, 0.0, angle),
            ),
            material=mount_steel,
            name=f"pl_mount_lug_{index}",
        )
    lens_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.057, length=0.160),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        save_mesh(
            "focus_ring_shell",
            cylindrical_shell(
                outer_radius=0.0558,
                inner_radius=0.0488,
                z0=0.054,
                z1=0.087,
                segments=88,
            ),
        ),
        material=satin_black,
        name="focus_ring_shell",
    )
    focus_ring.visual(
        save_mesh(
            "focus_ring_front_bead",
            cylindrical_shell(
                outer_radius=0.0568,
                inner_radius=0.0550,
                z0=0.054,
                z1=0.057,
                segments=88,
            ),
        ),
        material=satin_black,
        name="focus_ring_front_bead",
    )
    focus_ring.visual(
        save_mesh(
            "focus_ring_rear_bead",
            cylindrical_shell(
                outer_radius=0.0568,
                inner_radius=0.0550,
                z0=0.084,
                z1=0.087,
                segments=88,
            ),
        ),
        material=satin_black,
        name="focus_ring_rear_bead",
    )
    for index in range(22):
        angle = (2.0 * math.pi * index) / 22.0
        radius = 0.0566
        focus_ring.visual(
            Cylinder(radius=0.0019, length=0.029),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.0705),
            ),
            material=satin_black,
            name=f"focus_rib_{index}",
        )
    focus_marks = [
        (0.0565, 0.0045),
        (0.0625, 0.0065),
        (0.0690, 0.0090),
        (0.0760, 0.0065),
        (0.0820, 0.0045),
    ]
    for index, (z_pos, height) in enumerate(focus_marks):
        focus_ring.visual(
            Box((0.0016, 0.0026, height)),
            origin=Origin(xyz=(0.0, 0.0566, z_pos)),
            material=engraved_white,
            name=f"focus_mark_{index}",
        )
    focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0570, length=0.033),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.0705)),
    )

    pl_lock_ring = model.part("pl_lock_ring")
    pl_lock_ring.visual(
        save_mesh(
            "pl_lock_ring_shell",
            TorusGeometry(radius=0.0418, tube=0.0019, radial_segments=16, tubular_segments=56).translate(
                0.0,
                0.0,
                0.0035,
            ),
        ),
        material=mount_steel,
        name="pl_lock_ring_shell",
    )
    pl_lock_ring.visual(
        save_mesh(
            "pl_lock_ring_rear_band",
            TorusGeometry(radius=0.0418, tube=0.0019, radial_segments=16, tubular_segments=56).translate(
                0.0,
                0.0,
                0.0085,
            ),
        ),
        material=mount_steel,
        name="pl_lock_ring_rear_band",
    )
    for index in range(10):
        angle = (2.0 * math.pi * index) / 10.0
        radius = 0.0418
        pl_lock_ring.visual(
            Cylinder(radius=0.0013, length=0.007),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.006),
            ),
            material=mount_steel,
            name=f"pl_lock_rib_{index}",
        )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        radius = 0.0395
        pl_lock_ring.visual(
            Box((0.0030, 0.0050, 0.0050)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.006),
                rpy=(0.0, 0.0, angle),
            ),
            material=mount_steel,
            name=f"pl_lock_pad_{index}",
        )
    for index, x_pos in enumerate((0.0435, -0.0435)):
        pl_lock_ring.visual(
            Box((0.008, 0.012, 0.009)),
            origin=Origin(xyz=(x_pos, 0.0, 0.006)),
            material=mount_steel,
            name=f"pl_lock_grip_{index}",
        )
    pl_lock_ring.visual(
        Box((0.002, 0.0035, 0.004)),
        origin=Origin(xyz=(0.0, 0.0426, 0.006)),
        material=engraved_white,
        name="pl_alignment_mark",
    )
    pl_lock_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0435, length=0.012),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    model.articulation(
        "body_to_focus_ring",
        ArticulationType.REVOLUTE,
        parent=lens_body,
        child=focus_ring,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.5,
            lower=-2.6,
            upper=2.6,
        ),
    )
    model.articulation(
        "body_to_pl_lock_ring",
        ArticulationType.REVOLUTE,
        parent=lens_body,
        child=pl_lock_ring,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=2.0,
            lower=0.0,
            upper=0.62,
        ),
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
    lens_body = object_model.get_part("lens_body")
    focus_ring = object_model.get_part("focus_ring")
    pl_lock_ring = object_model.get_part("pl_lock_ring")
    focus_joint = object_model.get_articulation("body_to_focus_ring")
    pl_joint = object_model.get_articulation("body_to_pl_lock_ring")

    focus_limits = focus_joint.motion_limits
    pl_limits = pl_joint.motion_limits

    ctx.check(
        "focus ring rotates about optical axis",
        tuple(round(value, 6) for value in focus_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={focus_joint.axis}",
    )
    ctx.check(
        "PL locking ring rotates about optical axis",
        tuple(round(value, 6) for value in pl_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={pl_joint.axis}",
    )
    ctx.check(
        "focus ring has long cinema throw",
        focus_limits is not None
        and focus_limits.lower is not None
        and focus_limits.upper is not None
        and (focus_limits.upper - focus_limits.lower) >= 5.0,
        details=f"limits={focus_limits}",
    )
    ctx.check(
        "PL locking ring has short locking travel",
        pl_limits is not None
        and pl_limits.lower is not None
        and pl_limits.upper is not None
        and 0.2 <= (pl_limits.upper - pl_limits.lower) <= 1.0,
        details=f"limits={pl_limits}",
    )

    with ctx.pose({focus_joint: 0.0, pl_joint: 0.0}):
        ctx.expect_origin_distance(
            focus_ring,
            lens_body,
            axes="xy",
            max_dist=1e-6,
            name="focus ring is concentric with lens body",
        )
        ctx.expect_overlap(
            focus_ring,
            lens_body,
            axes="z",
            min_overlap=0.024,
            name="focus ring wraps around the main barrel",
        )
        ctx.expect_overlap(
            pl_lock_ring,
            lens_body,
            axes="z",
            min_overlap=0.008,
            name="PL locking ring sits around the rear mount collar",
        )
        ctx.expect_gap(
            focus_ring,
            pl_lock_ring,
            axis="z",
            min_gap=0.035,
            name="focus ring is forward of the PL lock mechanism",
        )

    with ctx.pose({focus_joint: 1.8, pl_joint: 0.45}):
        ctx.expect_origin_distance(
            focus_ring,
            lens_body,
            axes="xy",
            max_dist=1e-6,
            name="focus ring stays concentric when rotated",
        )
        ctx.expect_origin_distance(
            pl_lock_ring,
            lens_body,
            axes="xy",
            max_dist=1e-6,
            name="PL locking ring stays concentric when turned",
        )
        ctx.expect_overlap(
            focus_ring,
            lens_body,
            axes="z",
            min_overlap=0.024,
            name="focus ring remains supported through its travel",
        )
        ctx.expect_overlap(
            pl_lock_ring,
            lens_body,
            axes="z",
            min_overlap=0.008,
            name="PL locking ring remains mounted through lock travel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
