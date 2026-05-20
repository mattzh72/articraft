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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    boolean_difference,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observatory_dome")

    concrete = model.material("concrete", rgba=(0.70, 0.70, 0.72, 1.0))
    dome_white = model.material("dome_white", rgba=(0.93, 0.94, 0.95, 1.0))
    dark_track = model.material("dark_track", rgba=(0.20, 0.21, 0.23, 1.0))
    shutter_gray = model.material("shutter_gray", rgba=(0.80, 0.82, 0.84, 1.0))

    dome_outer_radius = 2.00
    dome_inner_radius = 1.965
    base_top_z = 0.58
    slit_width = 0.58
    hinge_z = 1.80
    hinge_x = math.sqrt((dome_outer_radius * dome_outer_radius) - (hinge_z * hinge_z))
    hinge_normal_x = hinge_x / dome_outer_radius
    hinge_normal_z = hinge_z / dome_outer_radius
    shutter_pitch = math.atan2(hinge_normal_x, hinge_normal_z)
    hinge_axis_offset = 0.065
    hinge_axis_x = hinge_x + (hinge_normal_x * hinge_axis_offset)
    hinge_axis_z = hinge_z + (hinge_normal_z * hinge_axis_offset)

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
            height=height + 0.01,
            radial_segments=radial_segments,
        )
        return boolean_difference(outer, inner).translate(0.0, 0.0, z_center)

    base = model.part("base_ring")
    base.visual(
        mesh_from_geometry(
            ring_band(
                outer_radius=2.28,
                inner_radius=1.90,
                height=0.58,
                z_center=0.29,
            ),
            "base_drum",
        ),
        material=concrete,
        name="base_drum",
    )
    base.visual(
        mesh_from_geometry(
            ring_band(
                outer_radius=2.08,
                inner_radius=1.92,
                height=0.06,
                z_center=0.55,
            ),
            "support_ring",
        ),
        material=dark_track,
        name="support_ring",
    )
    for name, xyz, size in (
        ("pier_east", (1.98, 0.0, 0.25), (0.74, 0.96, 0.50)),
        ("pier_west", (-1.98, 0.0, 0.25), (0.74, 0.96, 0.50)),
        ("pier_north", (0.0, 1.98, 0.25), (0.96, 0.74, 0.50)),
        ("pier_south", (0.0, -1.98, 0.25), (0.96, 0.74, 0.50)),
    ):
        base.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=concrete,
            name=name,
        )
    base.visual(
        mesh_from_geometry(
            TorusGeometry(radius=1.99, tube=0.03, radial_segments=18, tubular_segments=84),
            "roller_track",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.52)),
        material=dark_track,
        name="roller_track",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=2.30, length=0.58),
        mass=5200.0,
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
    )

    shell = model.part("dome_shell")

    dome_shell_geom = LatheGeometry.from_shell_profiles(
        [
            (0.04, 1.998),
            (0.30, 1.98),
            (0.72, 1.87),
            (1.14, 1.64),
            (1.50, 1.34),
            (1.76, 0.98),
            (1.94, 0.56),
            (2.02, 0.12),
            (2.02, 0.00),
        ],
        [
            (0.012, 1.965),
            (0.26, 1.95),
            (0.66, 1.84),
            (1.06, 1.62),
            (1.40, 1.33),
            (1.66, 0.98),
            (1.84, 0.58),
            (1.96, 0.16),
            (1.96, 0.00),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
    )
    slit_cutter = BoxGeometry((1.26, slit_width + 0.06, 0.90)).translate(1.25, 0.0, 1.55)
    dome_shell_geom = boolean_difference(dome_shell_geom, slit_cutter)
    dome_shell_geom.merge(ring_band(outer_radius=2.04, inner_radius=1.96, height=0.14, z_center=0.07))

    shell.visual(
        mesh_from_geometry(dome_shell_geom, "dome_shell"),
        material=dome_white,
        name="shell_body",
    )
    shell.visual(
        Box((0.18, 0.16, 0.22)),
        origin=Origin(
            xyz=(hinge_x + 0.03, 0.38, hinge_z + 0.015),
            rpy=(0.0, shutter_pitch, 0.0),
        ),
        material=dark_track,
        name="left_hinge_bracket",
    )
    shell.visual(
        Box((0.18, 0.16, 0.22)),
        origin=Origin(
            xyz=(hinge_x + 0.03, -0.38, hinge_z + 0.015),
            rpy=(0.0, shutter_pitch, 0.0),
        ),
        material=dark_track,
        name="right_hinge_bracket",
    )
    shell.inertial = Inertial.from_geometry(
        Cylinder(radius=2.04, length=2.00),
        mass=820.0,
        origin=Origin(xyz=(0.0, 0.0, 1.00)),
    )

    shutter = model.part("shutter_leaf")

    shutter_leaf_geom = BoxGeometry((0.84, 0.56, 0.03)).translate(-0.42, 0.0, 0.018)
    shutter_leaf_geom.rotate_y(shutter_pitch)
    shutter_leaf_geom.merge(
        BoxGeometry((0.18, 0.60, 0.12)).translate(-0.09, 0.0, 0.05).rotate_y(shutter_pitch)
    )
    shutter_leaf_geom.merge(CylinderGeometry(radius=0.045, height=0.60).rotate_x(math.pi / 2.0))
    shutter.visual(
        mesh_from_geometry(shutter_leaf_geom, "shutter_leaf"),
        material=shutter_gray,
        name="leaf_assembly",
    )
    shutter.inertial = Inertial.from_geometry(
        Box((0.82, 0.64, 0.18)),
        mass=70.0,
        origin=Origin(xyz=(0.30, 0.0, 0.03)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=shell,
        origin=Origin(xyz=(0.0, 0.0, base_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=0.35),
    )
    model.articulation(
        "slit_shutter",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=shutter,
        origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_axis_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=0.8,
            lower=0.0,
            upper=1.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_ring")
    shell = object_model.get_part("dome_shell")
    shutter = object_model.get_part("shutter_leaf")
    dome_rotation = object_model.get_articulation("azimuth_rotation")
    shutter_hinge = object_model.get_articulation("slit_shutter")

    ctx.expect_gap(
        shell,
        base,
        axis="z",
        positive_elem="shell_body",
        negative_elem="support_ring",
        max_gap=0.03,
        max_penetration=0.0,
        name="dome shell seats on support ring",
    )
    ctx.expect_overlap(
        shell,
        base,
        axes="xy",
        elem_a="shell_body",
        elem_b="base_drum",
        min_overlap=3.7,
        name="dome shell stays centered over the base ring",
    )

    with ctx.pose({dome_rotation: 1.2}):
        ctx.expect_overlap(
            shell,
            base,
            axes="xy",
            elem_a="shell_body",
            elem_b="base_drum",
            min_overlap=3.7,
            name="rotated shell remains centered over the base ring",
        )

    closed_aabb = ctx.part_world_aabb(shutter)
    with ctx.pose({shutter_hinge: shutter_hinge.motion_limits.upper}):
        opened_aabb = ctx.part_world_aabb(shutter)

    opens_outward = (
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][0] > closed_aabb[1][0] + 0.18
        and opened_aabb[1][2] > closed_aabb[1][2] + 0.10
    )
    ctx.check(
        "shutter leaf opens outward from the slit",
        opens_outward,
        details=f"closed_aabb={closed_aabb}, opened_aabb={opened_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
