from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    def ring_band(
        *,
        outer_radius: float,
        inner_radius: float,
        height: float,
        z_center: float,
        radial_segments: int = 72,
    ):
        outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=radial_segments)
        inner = CylinderGeometry(
            radius=inner_radius,
            height=height + 0.04,
            radial_segments=radial_segments,
        )
        return boolean_difference(outer, inner).translate(0.0, 0.0, z_center)

    def mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def shell_section(
        *,
        outer_radius: float,
        inner_radius: float,
        slit_width: float,
        z: float,
        arc_samples: int = 32,
    ):
        outer_half = min(slit_width * 0.5, outer_radius - 1e-4)
        inner_half = min(slit_width * 0.5, inner_radius - 1e-4)
        outer_phi = math.asin(outer_half / outer_radius)
        inner_phi = math.asin(inner_half / inner_radius)

        outer_angles = [
            outer_phi + (2.0 * math.pi - 2.0 * outer_phi) * i / (arc_samples - 1)
            for i in range(arc_samples)
        ]
        inner_angles = [
            (2.0 * math.pi - inner_phi) - (2.0 * math.pi - 2.0 * inner_phi) * i / (arc_samples - 1)
            for i in range(arc_samples)
        ]

        return [
            *[(outer_radius * math.cos(angle), outer_radius * math.sin(angle), z) for angle in outer_angles],
            *[(inner_radius * math.cos(angle), inner_radius * math.sin(angle), z) for angle in inner_angles],
        ]

    def build_dome_shell_mesh():
        return section_loft(
            [
                shell_section(outer_radius=1.72, inner_radius=1.60, slit_width=0.62, z=0.00),
                shell_section(outer_radius=1.72, inner_radius=1.60, slit_width=0.60, z=0.88),
                shell_section(outer_radius=1.67, inner_radius=1.56, slit_width=0.58, z=1.42),
                shell_section(outer_radius=1.54, inner_radius=1.45, slit_width=0.56, z=2.00),
                shell_section(outer_radius=1.30, inner_radius=1.22, slit_width=0.54, z=2.58),
                shell_section(outer_radius=0.96, inner_radius=0.88, slit_width=0.46, z=3.06),
                shell_section(outer_radius=0.55, inner_radius=0.47, slit_width=0.34, z=3.44),
                shell_section(outer_radius=0.24, inner_radius=0.16, slit_width=0.20, z=3.66),
            ]
        )

    def leaf_section(*, width: float, rise: float, outer_x: float, thickness: float):
        half_width = width * 0.5
        crown_x = outer_x - 0.020 - 0.010 * (rise / 1.10)
        inner_x = outer_x - thickness
        inner_crown_x = inner_x - 0.010
        return [
            (outer_x, -half_width, rise),
            (crown_x, 0.0, rise),
            (outer_x, half_width, rise),
            (inner_x, half_width, rise),
            (inner_crown_x, 0.0, rise),
            (inner_x, -half_width, rise),
        ]

    def build_shutter_leaf_mesh():
        return section_loft(
            [
                leaf_section(width=0.46, rise=0.00, outer_x=0.050, thickness=0.040),
                leaf_section(width=0.45, rise=0.32, outer_x=0.042, thickness=0.040),
                leaf_section(width=0.40, rise=0.72, outer_x=0.025, thickness=0.038),
                leaf_section(width=0.28, rise=1.08, outer_x=0.004, thickness=0.036),
            ]
        )

    model = ArticulatedObject(name="observatory_dome")

    concrete = model.material("concrete", rgba=(0.70, 0.71, 0.72, 1.0))
    steel = model.material("steel", rgba=(0.34, 0.37, 0.40, 1.0))
    dome_white = model.material("dome_white", rgba=(0.93, 0.94, 0.95, 1.0))
    graphite = model.material("graphite", rgba=(0.17, 0.18, 0.20, 1.0))

    base_ring = model.part("base_ring")
    base_ring.visual(
        mesh(
            "observatory_concrete_drum",
            ring_band(
                outer_radius=1.96,
                inner_radius=1.63,
                height=1.08,
                z_center=0.54,
                radial_segments=88,
            ),
        ),
        material=concrete,
        name="concrete_drum",
    )
    base_ring.visual(
        mesh(
            "observatory_track_ring",
            ring_band(
                outer_radius=1.83,
                inner_radius=1.66,
                height=0.11,
                z_center=1.10,
                radial_segments=88,
            ),
        ),
        material=steel,
        name="track_ring",
    )
    base_ring.visual(
        Cylinder(radius=2.08, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=concrete,
        name="foundation_pad",
    )
    base_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=2.08, length=1.20),
        mass=6200.0,
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
    )

    dome_shell = model.part("dome_shell")
    dome_shell.visual(
        mesh("observatory_dome_shell", build_dome_shell_mesh()),
        material=dome_white,
        name="shell_skin",
    )
    dome_shell.visual(
        mesh(
            "observatory_base_skirt_ring",
            ring_band(
                outer_radius=1.74,
                inner_radius=1.60,
                height=0.12,
                z_center=0.06,
                radial_segments=88,
            ),
        ),
        material=graphite,
        name="skirt_ring",
    )
    dome_shell.visual(
        Cylinder(radius=0.18, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 3.68)),
        material=graphite,
        name="top_hub",
    )
    dome_shell.visual(
        Box((0.06, 0.62, 0.04)),
        origin=Origin(xyz=(1.245, 0.0, 2.55)),
        material=graphite,
        name="shutter_header",
    )
    dome_shell.inertial = Inertial.from_geometry(
        Cylinder(radius=1.75, length=3.85),
        mass=2400.0,
        origin=Origin(xyz=(0.0, 0.0, 1.93)),
    )

    model.articulation(
        "base_to_dome",
        ArticulationType.CONTINUOUS,
        parent=base_ring,
        child=dome_shell,
        origin=Origin(xyz=(0.0, 0.0, 1.155)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=0.35),
    )

    shutter_leaf = model.part("shutter_leaf")
    shutter_leaf.visual(
        mesh("observatory_shutter_leaf", build_shutter_leaf_mesh()),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dome_white,
        name="leaf_panel",
    )
    shutter_leaf.inertial = Inertial.from_geometry(
        Box((0.10, 0.50, 1.15)),
        mass=95.0,
        origin=Origin(xyz=(0.01, 0.0, 0.54)),
    )

    model.articulation(
        "dome_to_shutter",
        ArticulationType.REVOLUTE,
        parent=dome_shell,
        child=shutter_leaf,
        origin=Origin(xyz=(1.22, 0.0, 2.57)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.45,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    dome_shell = object_model.get_part("dome_shell")
    shutter_leaf = object_model.get_part("shutter_leaf")
    shutter_joint = object_model.get_articulation("dome_to_shutter")

    with ctx.pose({shutter_joint: 0.0}):
        ctx.expect_contact(
            shutter_leaf,
            dome_shell,
            elem_a="leaf_panel",
            elem_b="shutter_header",
            contact_tol=1e-6,
            name="closed shutter leaf seats on the shutter header",
        )
        ctx.expect_overlap(
            shutter_leaf,
            dome_shell,
            axes="y",
            elem_a="leaf_panel",
            elem_b="shutter_header",
            min_overlap=0.44,
            name="closed shutter spans the shutter header width",
        )

    with ctx.pose({shutter_joint: 0.0}):
        closed_leaf_aabb = ctx.part_element_world_aabb(
            shutter_leaf,
            elem="leaf_panel",
        )
    with ctx.pose({shutter_joint: 1.20}):
        open_leaf_aabb = ctx.part_element_world_aabb(
            shutter_leaf,
            elem="leaf_panel",
        )

    ctx.check(
        "shutter opens outward from the dome crown",
        closed_leaf_aabb is not None
        and open_leaf_aabb is not None
        and open_leaf_aabb[1][0] > closed_leaf_aabb[1][0] + 0.75,
        details=f"closed={closed_leaf_aabb}, open={open_leaf_aabb}",
    )

    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
