from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


HUB_Z = 0.18
HINGE_R = 0.105
LEG_REACH = 0.50
LEG_DROP = 0.16
LEG_LENGTH = math.sqrt(LEG_REACH * LEG_REACH + LEG_DROP * LEG_DROP)
LEG_PITCH = math.atan2(LEG_REACH, -LEG_DROP)
LEG_ANGLES = (math.radians(90.0), math.radians(210.0), math.radians(330.0))


def _radial_xy(radius: float, angle: float, tangent_offset: float = 0.0) -> tuple[float, float]:
    """Point in the floor-lamp radial/tangent frame."""
    return (
        radius * math.cos(angle) - tangent_offset * math.sin(angle),
        radius * math.sin(angle) + tangent_offset * math.cos(angle),
    )


def _shade_shell_mesh():
    """Thin lathed upward-facing bowl shade with an integral thick rim."""
    return LatheGeometry.from_shell_profiles(
        [
            (0.070, 1.430),
            (0.105, 1.470),
            (0.185, 1.560),
            (0.275, 1.650),
            (0.335, 1.715),
        ],
        [
            (0.052, 1.445),
            (0.090, 1.485),
            (0.165, 1.565),
            (0.245, 1.645),
            (0.302, 1.690),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_floor_lamp")

    satin_black = model.material("satin_black", rgba=(0.02, 0.022, 0.024, 1.0))
    warm_brass = model.material("warm_brass", rgba=(0.72, 0.56, 0.30, 1.0))
    shade_ivory = model.material("shade_ivory", rgba=(0.90, 0.86, 0.74, 1.0))
    warm_glass = model.material("warm_glass", rgba=(1.0, 0.82, 0.45, 0.72))
    rubber = model.material("dark_rubber", rgba=(0.006, 0.006, 0.006, 1.0))

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.078, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.117)),
        material=satin_black,
        name="hub_drum",
    )
    hub.visual(
        Cylinder(radius=0.052, length=1.315),
        origin=Origin(xyz=(0.0, 0.0, 0.805)),
        material=satin_black,
        name="column",
    )
    hub.visual(
        Cylinder(radius=0.070, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 1.440)),
        material=warm_brass,
        name="socket_cup",
    )
    hub.visual(
        Cylinder(radius=0.092, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 1.455)),
        material=warm_brass,
        name="shade_collar",
    )
    hub.visual(
        Cylinder(radius=0.012, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 1.500)),
        material=warm_brass,
        name="bulb_stem",
    )
    hub.visual(
        Sphere(radius=0.047),
        origin=Origin(xyz=(0.0, 0.0, 1.555)),
        material=warm_glass,
        name="bulb",
    )
    hub.visual(
        mesh_from_geometry(_shade_shell_mesh(), "bowl_shade"),
        material=shade_ivory,
        name="shade_shell",
    )

    for index, angle in enumerate(LEG_ANGLES):
        yaw = angle
        # Parent-side clevis ears flank the moving hinge barrel without occupying it.
        for side in (-1.0, 1.0):
            x, y = _radial_xy(HINGE_R, angle, side * 0.050)
            hub.visual(
                Box((0.082, 0.014, 0.068)),
                origin=Origin(xyz=(x, y, HUB_Z), rpy=(0.0, 0.0, yaw)),
                material=satin_black,
                name=f"hinge_ear_{index}_{'a' if side < 0 else 'b'}",
            )
            cap_x, cap_y = _radial_xy(HINGE_R, angle, side * 0.062)
            hub.visual(
                Cylinder(radius=0.016, length=0.010),
                origin=Origin(xyz=(cap_x, cap_y, HUB_Z), rpy=(-math.pi / 2.0, 0.0, yaw)),
                material=warm_brass,
                name=f"hinge_pin_cap_{index}_{'a' if side < 0 else 'b'}",
            )

        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.018, length=0.086),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=warm_brass,
            name="hinge_barrel",
        )
        leg.visual(
            Cylinder(radius=0.014, length=LEG_LENGTH),
            origin=Origin(
                xyz=(LEG_REACH / 2.0, 0.0, -LEG_DROP / 2.0),
                rpy=(0.0, LEG_PITCH, 0.0),
            ),
            material=satin_black,
            name="spar",
        )
        leg.visual(
            Cylinder(radius=0.046, length=0.020),
            origin=Origin(xyz=(LEG_REACH, 0.0, -LEG_DROP)),
            material=rubber,
            name="foot_pad",
        )

        hinge_x, hinge_y = _radial_xy(HINGE_R, angle)
        model.articulation(
            f"hub_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=hub,
            child=leg,
            origin=Origin(xyz=(hinge_x, hinge_y, HUB_Z), rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.35, upper=0.25),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hub = object_model.get_part("hub")

    leg_joints = [object_model.get_articulation(f"hub_to_leg_{i}") for i in range(3)]
    ctx.check(
        "three revolute folding leg hinges",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in leg_joints),
        details=f"joints={leg_joints}",
    )

    column_box = ctx.part_element_world_aabb(hub, elem="column")
    shade_box = ctx.part_element_world_aabb(hub, elem="shade_shell")
    if column_box is not None and shade_box is not None:
        column_height = column_box[1][2] - column_box[0][2]
        shade_width = max(shade_box[1][0] - shade_box[0][0], shade_box[1][1] - shade_box[0][1])
        shade_above_column = shade_box[0][2] > column_box[0][2] + 1.20
    else:
        column_height = 0.0
        shade_width = 0.0
        shade_above_column = False
    ctx.check(
        "tall column with wide bowl shade",
        column_height > 1.20 and shade_width > 0.58 and shade_above_column,
        details=f"column_height={column_height:.3f}, shade_width={shade_width:.3f}",
    )

    for index, joint in enumerate(leg_joints):
        leg = object_model.get_part(f"leg_{index}")
        foot_box = ctx.part_element_world_aabb(leg, elem="foot_pad")
        rest_center_z = None if foot_box is None else 0.5 * (foot_box[0][2] + foot_box[1][2])
        with ctx.pose({joint: -0.35}):
            folded_box = ctx.part_element_world_aabb(leg, elem="foot_pad")
            folded_center_z = None if folded_box is None else 0.5 * (folded_box[0][2] + folded_box[1][2])
        ctx.check(
            f"leg_{index} folds upward about hub hinge",
            rest_center_z is not None
            and folded_center_z is not None
            and folded_center_z > rest_center_z + 0.10,
            details=f"rest_z={rest_center_z}, folded_z={folded_center_z}",
        )

    return ctx.report()


object_model = build_object_model()
