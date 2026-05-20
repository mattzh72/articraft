from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BARREL_RADIUS = 0.0032
BARREL_LENGTH = 0.0062
ARM_THICKNESS = 0.0018
JOINT_SIDE_CLEARANCE = 0.0003
KNUCKLE_WIDTH = BARREL_LENGTH
OUTER_WIDTH = BARREL_LENGTH + 2.0 * ARM_THICKNESS
HINGE_LENGTH = 0.0036
SADDLE_LENGTH = 0.0010
FORK_REAR = 0.0036
FORK_FRONT = 0.0030

ROOT_BACK_LENGTH = 0.018
ROOT_THICKNESS = 0.0080

PROXIMAL_LENGTH = 0.034
MIDDLE_LENGTH = 0.027
DISTAL_LENGTH = 0.024
PAD_LENGTH = 0.010

NECK_END = 0.0062
NECK_WIDTH = 0.0050


def _x_box(x0: float, x1: float, width: float, thickness: float) -> cq.Workplane:
    return cq.Workplane("XY").box(x1 - x0, width, thickness).translate(
        ((x0 + x1) / 2.0, 0.0, 0.0)
    )


def _x_box_at(
    x0: float,
    x1: float,
    width: float,
    thickness: float,
    *,
    y_center: float = 0.0,
) -> cq.Workplane:
    return _x_box(x0, x1, width, thickness).translate((0.0, y_center, 0.0))


def _y_cylinder(radius: float, length: float, x_center: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length).translate(
        (x_center, -length / 2.0, 0.0)
    )


def _beam(
    x0: float,
    x1: float,
    root_width: float,
    tip_width: float,
    root_thickness: float,
    tip_thickness: float,
) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .rect(root_width, root_thickness)
        .workplane(offset=x1 - x0)
        .rect(tip_width, tip_thickness)
        .loft(combine=True)
        .translate((x0, 0.0, 0.0))
    )


def _fork(x0: float, x1: float, thickness: float) -> cq.Workplane:
    arm_offset = BARREL_LENGTH / 2.0 + ARM_THICKNESS / 2.0
    left = _x_box_at(x0, x1, ARM_THICKNESS, thickness, y_center=arm_offset)
    right = _x_box_at(x0, x1, ARM_THICKNESS, thickness, y_center=-arm_offset)
    return left.union(right)


def _knuckle(x0: float, x1: float, thickness: float) -> cq.Workplane:
    return _x_box(x0, x1, KNUCKLE_WIDTH, thickness)


def _root_clevis_shape() -> cq.Workplane:
    spine = _x_box(-ROOT_BACK_LENGTH, -HINGE_LENGTH, OUTER_WIDTH, ROOT_THICKNESS)
    taper = _beam(-0.010, -HINGE_LENGTH, OUTER_WIDTH * 0.92, OUTER_WIDTH * 0.74, 0.0076, 0.0060)
    bridge = _x_box(-0.0068, -HINGE_LENGTH, OUTER_WIDTH * 0.74, 0.0060)
    fork = _fork(-HINGE_LENGTH, 0.0, 0.0060)
    saddle = _x_box(-SADDLE_LENGTH, 0.0, BARREL_LENGTH + 0.0002, 0.0046)
    return spine.union(taper).union(bridge).union(fork).union(saddle)


def _center_link_shape(
    length: float,
    *,
    root_width: float,
    tip_width: float,
    root_thickness: float,
    tip_thickness: float,
) -> cq.Workplane:
    rear_knuckle = _x_box(0.0, HINGE_LENGTH, BARREL_LENGTH - 2.0 * JOINT_SIDE_CLEARANCE, root_thickness)
    neck = _x_box(HINGE_LENGTH, NECK_END, NECK_WIDTH, root_thickness * 0.88)
    finger = _beam(NECK_END - 0.0004, length - HINGE_LENGTH - 0.0016, root_width, tip_width, root_thickness, tip_thickness)
    shoulder = _x_box(length - HINGE_LENGTH - 0.0016, length - HINGE_LENGTH, OUTER_WIDTH * 0.70, tip_thickness)
    fork = _fork(length - HINGE_LENGTH, length, tip_thickness)
    saddle = _x_box(length - SADDLE_LENGTH, length, BARREL_LENGTH + 0.0002, tip_thickness * 0.92)
    return rear_knuckle.union(neck).union(finger).union(shoulder).union(fork).union(saddle)


def _distal_link_shape() -> cq.Workplane:
    rear_knuckle = _x_box(0.0, HINGE_LENGTH, BARREL_LENGTH - 2.0 * JOINT_SIDE_CLEARANCE, 0.0051)
    neck = _x_box(HINGE_LENGTH, NECK_END, NECK_WIDTH, 0.0051)
    finger = _beam(NECK_END - 0.0004, DISTAL_LENGTH, 0.0082, 0.0065, 0.0058, 0.0048)
    pad_stem = _x_box(DISTAL_LENGTH - 0.0006, DISTAL_LENGTH + 0.0045, 0.0062, 0.0048)
    pad_nose = _y_cylinder(0.0026, 0.0056, DISTAL_LENGTH + PAD_LENGTH * 0.55)
    return rear_knuckle.union(neck).union(finger).union(pad_stem).union(pad_nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_phalanx_chain")

    root_finish = model.material("root_finish", rgba=(0.21, 0.23, 0.26, 1.0))
    link_finish = model.material("link_finish", rgba=(0.70, 0.73, 0.77, 1.0))
    tip_finish = model.material("tip_finish", rgba=(0.60, 0.63, 0.67, 1.0))

    root = model.part("root_clevis")
    root.visual(
        mesh_from_cadquery(_root_clevis_shape(), "root_clevis"),
        material=root_finish,
        name="root_shell",
    )
    root.inertial = Inertial.from_geometry(
        Box((ROOT_BACK_LENGTH + FORK_FRONT, OUTER_WIDTH, ROOT_THICKNESS)),
        mass=0.05,
        origin=Origin(xyz=((-ROOT_BACK_LENGTH + FORK_FRONT) / 2.0, 0.0, 0.0)),
    )

    proximal = model.part("proximal_link")
    proximal.visual(
        mesh_from_cadquery(
            _center_link_shape(
                PROXIMAL_LENGTH,
                root_width=0.0105,
                tip_width=0.0088,
                root_thickness=0.0060,
                tip_thickness=0.0054,
            ),
            "proximal_link",
        ),
        material=link_finish,
        name="proximal_shell",
    )
    proximal.inertial = Inertial.from_geometry(
        Box((PROXIMAL_LENGTH + FORK_FRONT, 0.0105, 0.0060)),
        mass=0.03,
        origin=Origin(xyz=((PROXIMAL_LENGTH + FORK_FRONT) / 2.0, 0.0, 0.0)),
    )

    middle = model.part("middle_link")
    middle.visual(
        mesh_from_cadquery(
            _center_link_shape(
                MIDDLE_LENGTH,
                root_width=0.0088,
                tip_width=0.0074,
                root_thickness=0.0054,
                tip_thickness=0.0049,
            ),
            "middle_link",
        ),
        material=link_finish,
        name="middle_shell",
    )
    middle.inertial = Inertial.from_geometry(
        Box((MIDDLE_LENGTH + FORK_FRONT, 0.0088, 0.0054)),
        mass=0.022,
        origin=Origin(xyz=((MIDDLE_LENGTH + FORK_FRONT) / 2.0, 0.0, 0.0)),
    )

    distal = model.part("distal_link")
    distal.visual(
        mesh_from_cadquery(_distal_link_shape(), "distal_link"),
        material=tip_finish,
        name="distal_shell",
    )
    distal.inertial = Inertial.from_geometry(
        Box((DISTAL_LENGTH + PAD_LENGTH, 0.0082, 0.0058)),
        mass=0.015,
        origin=Origin(xyz=((DISTAL_LENGTH + PAD_LENGTH) / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "root_to_proximal",
        ArticulationType.REVOLUTE,
        parent=root,
        child=proximal,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=1.2),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(MIDDLE_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=1.1),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_clevis")
    proximal = object_model.get_part("proximal_link")
    middle = object_model.get_part("middle_link")
    distal = object_model.get_part("distal_link")
    root_to_proximal = object_model.get_articulation("root_to_proximal")
    proximal_to_middle = object_model.get_articulation("proximal_to_middle")
    middle_to_distal = object_model.get_articulation("middle_to_distal")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    for hinge in (root_to_proximal, proximal_to_middle, middle_to_distal):
        ctx.check(
            f"{hinge.name}_axis_is_y",
            hinge.axis == (0.0, 1.0, 0.0),
            details=f"axis={hinge.axis}",
        )

    rest_tip = ctx.part_world_position(distal)
    with ctx.pose(
        {
            root_to_proximal: 0.55,
            proximal_to_middle: 0.65,
            middle_to_distal: 0.55,
        }
    ):
        curled_tip = ctx.part_world_position(distal)
        ctx.check(
            "three-joint chain curls in one plane",
            rest_tip is not None
            and curled_tip is not None
            and curled_tip[2] < rest_tip[2] - 0.015
            and curled_tip[0] < rest_tip[0] - 0.010
            and abs(curled_tip[1] - rest_tip[1]) < 0.002,
            details=f"rest_tip={rest_tip}, curled_tip={curled_tip}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
