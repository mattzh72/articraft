from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


VANE_CENTERS = (-0.30, -0.18, -0.06, 0.06, 0.18, 0.30)
VANE_PRESET_ANGLE = math.radians(22.0)


def _airfoil_vane_mesh():
    """Slim vertical louver blade with a rounded nose and tapered tail."""
    profile = [
        (-0.038, -0.006),
        (-0.030, -0.011),
        (0.040, -0.009),
        (0.071, -0.003),
        (0.074, 0.000),
        (0.071, 0.003),
        (0.040, 0.009),
        (-0.030, 0.011),
        (-0.038, 0.006),
        (-0.043, 0.000),
    ]
    return ExtrudeGeometry.centered(profile, 0.45, cap=True, closed=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pivoting_vane_array")

    frame_paint = model.material("powder_coated_frame", rgba=(0.08, 0.09, 0.10, 1.0))
    edge_wear = model.material("worn_black_edges", rgba=(0.16, 0.17, 0.18, 1.0))
    bearing = model.material("dark_bushing", rgba=(0.03, 0.035, 0.04, 1.0))
    blade_metal = model.material("satin_aluminum", rgba=(0.72, 0.76, 0.78, 1.0))
    shaft_steel = model.material("polished_pivot_shafts", rgba=(0.55, 0.57, 0.58, 1.0))
    fastener = model.material("fastener_heads", rgba=(0.25, 0.26, 0.27, 1.0))

    frame = model.part("support_frame")
    # Rigid rectangular perimeter; the inner aperture is open and the vanes sit inside it.
    frame.visual(
        Box((0.94, 0.12, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
        material=frame_paint,
        name="top_rail",
    )
    frame.visual(
        Box((0.94, 0.12, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=frame_paint,
        name="bottom_rail",
    )
    frame.visual(
        Box((0.08, 0.12, 0.72)),
        origin=Origin(xyz=(-0.43, 0.0, 0.36)),
        material=frame_paint,
        name="side_rail_0",
    )
    frame.visual(
        Box((0.08, 0.12, 0.72)),
        origin=Origin(xyz=(0.43, 0.0, 0.36)),
        material=frame_paint,
        name="side_rail_1",
    )
    frame.visual(
        Box((0.86, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.069, 0.635)),
        material=edge_wear,
        name="rear_top_lip",
    )
    frame.visual(
        Box((0.86, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.069, 0.085)),
        material=edge_wear,
        name="rear_bottom_lip",
    )

    for x in VANE_CENTERS:
        # Paired top and bottom clevis lugs physically tied into the rails.
        # They leave a clear central slot for the rotating shaft.
        for y in (-0.046, 0.046):
            frame.visual(
                Box((0.050, 0.020, 0.052)),
                origin=Origin(xyz=(x, y, 0.618)),
                material=bearing,
                name=f"top_bearing_{x:+.2f}_{'front' if y > 0 else 'rear'}",
            )
            frame.visual(
                Box((0.050, 0.020, 0.052)),
                origin=Origin(xyz=(x, y, 0.102)),
                material=bearing,
                name=f"bottom_bearing_{x:+.2f}_{'front' if y > 0 else 'rear'}",
            )
        frame.visual(
            Cylinder(radius=0.010, length=0.012),
            origin=Origin(xyz=(x - 0.030, 0.061, 0.675)),
            material=fastener,
            name=f"top_screw_{x:+.2f}",
        )
        frame.visual(
            Cylinder(radius=0.010, length=0.012),
            origin=Origin(xyz=(x + 0.030, 0.061, 0.045)),
            material=fastener,
            name=f"bottom_screw_{x:+.2f}",
        )
        # Small stop ears near the lower rail show the limited sweep of each vane.
        frame.visual(
            Box((0.018, 0.018, 0.040)),
            origin=Origin(xyz=(x - 0.042, -0.030, 0.098)),
            material=edge_wear,
            name=f"stop_ear_{x:+.2f}_0",
        )
        frame.visual(
            Box((0.018, 0.018, 0.040)),
            origin=Origin(xyz=(x + 0.042, 0.030, 0.098)),
            material=edge_wear,
            name=f"stop_ear_{x:+.2f}_1",
        )

    frame.inertial = Inertial.from_geometry(
        Box((0.94, 0.14, 0.72)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
    )

    blade_mesh = mesh_from_geometry(_airfoil_vane_mesh(), "slim_airfoil_vane")
    for index, x in enumerate(VANE_CENTERS):
        vane = model.part(f"vane_{index}")
        vane.visual(
            Cylinder(radius=0.0085, length=0.560),
            origin=Origin(),
            material=shaft_steel,
            name="pivot_shaft",
        )
        vane.visual(
            blade_mesh,
            origin=Origin(rpy=(0.0, 0.0, VANE_PRESET_ANGLE)),
            material=blade_metal,
            name="blade_shell",
        )
        vane.visual(
            Box((0.030, 0.014, 0.022)),
            origin=Origin(xyz=(0.0, 0.0, -0.246), rpy=(0.0, 0.0, VANE_PRESET_ANGLE)),
            material=blade_metal,
            name="lower_hub",
        )
        vane.visual(
            Box((0.030, 0.014, 0.022)),
            origin=Origin(xyz=(0.0, 0.0, 0.246), rpy=(0.0, 0.0, VANE_PRESET_ANGLE)),
            material=blade_metal,
            name="upper_hub",
        )
        vane.inertial = Inertial.from_geometry(
            Box((0.10, 0.025, 0.54)),
            mass=0.22,
            origin=Origin(),
        )
        model.articulation(
            f"vane_{index}_pivot",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=vane,
            origin=Origin(xyz=(x, 0.0, 0.36)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.2, velocity=2.0, lower=-0.75, upper=0.75),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("support_frame")
    vane_joints = [object_model.get_articulation(f"vane_{i}_pivot") for i in range(len(VANE_CENTERS))]

    ctx.check(
        "six independent vane pivots",
        len(vane_joints) == 6
        and all(joint.motion_limits.lower <= -0.70 and joint.motion_limits.upper >= 0.70 for joint in vane_joints),
        details="Expected six revolute vane pivots with roughly +/-43 degree travel.",
    )

    for index, joint in enumerate(vane_joints):
        vane = object_model.get_part(f"vane_{index}")
        ctx.expect_within(
            vane,
            frame,
            axes="xz",
            margin=0.002,
            name=f"vane_{index} remains inside the frame opening",
        )
        ctx.expect_gap(
            vane,
            frame,
            axis="z",
            max_gap=0.001,
            max_penetration=0.00001,
            positive_elem="pivot_shaft",
            negative_elem="bottom_rail",
            name=f"vane_{index} lower shaft end seats at the bottom rail",
        )
        ctx.expect_gap(
            frame,
            vane,
            axis="z",
            max_gap=0.001,
            max_penetration=0.00001,
            positive_elem="top_rail",
            negative_elem="pivot_shaft",
            name=f"vane_{index} upper shaft end seats at the top rail",
        )
        rest_aabb = ctx.part_world_aabb(vane)
        with ctx.pose({joint: 0.55}):
            swept_aabb = ctx.part_world_aabb(vane)
        ctx.check(
            f"vane_{index} visibly rotates about its shaft",
            rest_aabb is not None
            and swept_aabb is not None
            and abs((swept_aabb[1][1] - swept_aabb[0][1]) - (rest_aabb[1][1] - rest_aabb[0][1])) > 0.020,
            details=f"rest_aabb={rest_aabb}, swept_aabb={swept_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
