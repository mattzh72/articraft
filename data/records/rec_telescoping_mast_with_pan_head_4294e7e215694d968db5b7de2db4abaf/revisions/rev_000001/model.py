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


BASE_PLATE_SIZE = 0.34
BASE_PLATE_THICKNESS = 0.03

LOWER_OUTER = 0.18
LOWER_INNER = 0.158
LOWER_HEIGHT = 0.62
LOWER_COLLAR_OUTER = 0.225
LOWER_COLLAR_INNER = 0.186
LOWER_COLLAR_HEIGHT = 0.05
LOWER_GUIDE_OUTER = 0.205
LOWER_GUIDE_INNER = 0.154
LOWER_GUIDE_HEIGHT = 0.03
BASE_BRACE_LENGTH = 0.15
BASE_BRACE_THICKNESS = 0.018
BASE_BRACE_HEIGHT = 0.16

MIDDLE_OUTER = 0.148
MIDDLE_INNER = 0.130
MIDDLE_TOTAL = 0.88
MIDDLE_INSERT = 0.32
MIDDLE_TRAVEL = 0.20
MIDDLE_HEAD_OUTER = 0.165
MIDDLE_HEAD_INNER = 0.124
MIDDLE_HEAD_HEIGHT = 0.03
STAGE_STOP_HEIGHT = 0.02
STAGE_STOP_Z = 0.026
MIDDLE_STOP_OUTER = 0.19
MIDDLE_STOP_INNER = 0.146
UPPER_STOP_OUTER = 0.142
UPPER_STOP_INNER = 0.116

UPPER_OUTER = 0.118
UPPER_INNER = 0.102
UPPER_TOTAL = 0.68
UPPER_INSERT = 0.26
UPPER_TRAVEL = 0.14
UPPER_MOUNT_PLATE = 0.15
UPPER_MOUNT_THICKNESS = 0.012

CARTRIDGE_BASE_RADIUS = 0.065
CARTRIDGE_BASE_HEIGHT = 0.038
CARTRIDGE_TOP_LENGTH = 0.16
CARTRIDGE_TOP_WIDTH = 0.14
CARTRIDGE_TOP_THICKNESS = 0.012
CARTRIDGE_CAP_RADIUS = 0.042
CARTRIDGE_CAP_HEIGHT = 0.018
CARTRIDGE_POD_LENGTH = 0.06
CARTRIDGE_POD_WIDTH = 0.038
CARTRIDGE_POD_HEIGHT = 0.044
CARTRIDGE_POD_X = 0.088


def _square_tube(outer: float, inner: float, length: float) -> cq.Workplane:
    return cq.Workplane("XY").rect(outer, outer).rect(inner, inner).extrude(length)


def _box(
    x_size: float,
    y_size: float,
    z_size: float,
    *,
    z0: float = 0.0,
    x0: float = 0.0,
    y0: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(x_size, y_size, z_size, centered=(True, True, False))
        .translate((x0, y0, z0))
    )


def _lower_sleeve_shell_shape() -> cq.Workplane:
    return _square_tube(LOWER_OUTER, LOWER_INNER, LOWER_HEIGHT).translate(
        (0.0, 0.0, BASE_PLATE_THICKNESS)
    )


def _lower_base_structure_shape() -> cq.Workplane:
    plate = _box(BASE_PLATE_SIZE, BASE_PLATE_SIZE, BASE_PLATE_THICKNESS)
    collar = (
        _square_tube(LOWER_COLLAR_OUTER, LOWER_COLLAR_INNER, LOWER_COLLAR_HEIGHT)
        .translate((0.0, 0.0, BASE_PLATE_THICKNESS))
    )
    north_brace = _box(
        BASE_BRACE_LENGTH,
        BASE_BRACE_THICKNESS,
        BASE_BRACE_HEIGHT,
        z0=BASE_PLATE_THICKNESS,
        y0=(LOWER_COLLAR_OUTER / 2.0) + (BASE_BRACE_THICKNESS / 2.0),
    )
    south_brace = _box(
        BASE_BRACE_LENGTH,
        BASE_BRACE_THICKNESS,
        BASE_BRACE_HEIGHT,
        z0=BASE_PLATE_THICKNESS,
        y0=-((LOWER_COLLAR_OUTER / 2.0) + (BASE_BRACE_THICKNESS / 2.0)),
    )
    east_brace = _box(
        BASE_BRACE_THICKNESS,
        BASE_BRACE_LENGTH,
        BASE_BRACE_HEIGHT,
        z0=BASE_PLATE_THICKNESS,
        x0=(LOWER_COLLAR_OUTER / 2.0) + (BASE_BRACE_THICKNESS / 2.0),
    )
    west_brace = _box(
        BASE_BRACE_THICKNESS,
        BASE_BRACE_LENGTH,
        BASE_BRACE_HEIGHT,
        z0=BASE_PLATE_THICKNESS,
        x0=-((LOWER_COLLAR_OUTER / 2.0) + (BASE_BRACE_THICKNESS / 2.0)),
    )
    return (
        plate.union(collar)
        .union(north_brace)
        .union(south_brace)
        .union(east_brace)
        .union(west_brace)
    )


def _lower_guide_shape() -> cq.Workplane:
    return _square_tube(LOWER_GUIDE_OUTER, LOWER_GUIDE_INNER, LOWER_GUIDE_HEIGHT).translate(
        (0.0, 0.0, BASE_PLATE_THICKNESS + LOWER_HEIGHT - 0.004)
    )


def _middle_stage_shell_shape() -> cq.Workplane:
    return _square_tube(MIDDLE_OUTER, MIDDLE_INNER, MIDDLE_TOTAL).translate(
        (0.0, 0.0, -MIDDLE_INSERT)
    )


def _middle_stage_head_shape() -> cq.Workplane:
    return _square_tube(MIDDLE_HEAD_OUTER, MIDDLE_HEAD_INNER, MIDDLE_HEAD_HEIGHT).translate(
        (0.0, 0.0, MIDDLE_TOTAL - MIDDLE_INSERT - 0.004)
    )


def _middle_stage_stop_collar_shape() -> cq.Workplane:
    return _square_tube(MIDDLE_STOP_OUTER, MIDDLE_STOP_INNER, STAGE_STOP_HEIGHT).translate(
        (0.0, 0.0, STAGE_STOP_Z)
    )


def _upper_stage_shell_shape() -> cq.Workplane:
    return _square_tube(UPPER_OUTER, UPPER_INNER, UPPER_TOTAL).translate(
        (0.0, 0.0, -UPPER_INSERT)
    )


def _upper_stage_stop_collar_shape() -> cq.Workplane:
    return _square_tube(UPPER_STOP_OUTER, UPPER_STOP_INNER, STAGE_STOP_HEIGHT).translate(
        (0.0, 0.0, STAGE_STOP_Z)
    )


def _upper_stage_mount_shape() -> cq.Workplane:
    upper_exposed = UPPER_TOTAL - UPPER_INSERT
    return _box(
        UPPER_MOUNT_PLATE,
        UPPER_MOUNT_PLATE,
        UPPER_MOUNT_THICKNESS,
        z0=upper_exposed - 0.004,
    )


def _cartridge_base_shape() -> cq.Workplane:
    return cq.Workplane("XY").circle(CARTRIDGE_BASE_RADIUS).extrude(CARTRIDGE_BASE_HEIGHT)


def _cartridge_top_plate_shape() -> cq.Workplane:
    return _box(
        CARTRIDGE_TOP_LENGTH,
        CARTRIDGE_TOP_WIDTH,
        CARTRIDGE_TOP_THICKNESS,
        z0=CARTRIDGE_BASE_HEIGHT - 0.003,
    )


def _cartridge_cap_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(CARTRIDGE_CAP_RADIUS)
        .extrude(CARTRIDGE_CAP_HEIGHT)
        .translate((0.0, 0.0, CARTRIDGE_BASE_HEIGHT + CARTRIDGE_TOP_THICKNESS - 0.004))
    )


def _cartridge_side_pod_shape() -> cq.Workplane:
    return _box(
        CARTRIDGE_POD_LENGTH,
        CARTRIDGE_POD_WIDTH,
        CARTRIDGE_POD_HEIGHT,
        x0=CARTRIDGE_POD_X,
        z0=0.008,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_mast_with_pan_cartridge")

    model.material("powder_coat_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("anodized_gray", rgba=(0.66, 0.69, 0.73, 1.0))
    model.material("anodized_light", rgba=(0.78, 0.80, 0.83, 1.0))
    model.material("machined_silver", rgba=(0.80, 0.82, 0.85, 1.0))
    model.material("service_black", rgba=(0.10, 0.11, 0.12, 1.0))

    lower_sleeve = model.part("lower_sleeve")
    lower_sleeve.visual(
        mesh_from_cadquery(_lower_base_structure_shape(), "lower_base_structure_mesh"),
        material="powder_coat_dark",
        name="lower_base_structure",
    )
    lower_sleeve.visual(
        mesh_from_cadquery(_lower_sleeve_shell_shape(), "lower_sleeve_shell_mesh"),
        material="powder_coat_dark",
        name="lower_sleeve_shell",
    )
    lower_sleeve.visual(
        mesh_from_cadquery(_lower_guide_shape(), "lower_sleeve_guide_mesh"),
        material="service_black",
        name="lower_sleeve_guide",
    )
    lower_sleeve.inertial = Inertial.from_geometry(
        Box((BASE_PLATE_SIZE, BASE_PLATE_SIZE, BASE_PLATE_THICKNESS + LOWER_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_PLATE_THICKNESS + LOWER_HEIGHT) / 2.0)),
    )

    middle_stage = model.part("middle_stage")
    middle_stage.visual(
        mesh_from_cadquery(_middle_stage_shell_shape(), "middle_stage_shell_mesh"),
        material="anodized_gray",
        name="middle_stage_shell",
    )
    middle_stage.visual(
        mesh_from_cadquery(_middle_stage_head_shape(), "middle_stage_head_mesh"),
        material="service_black",
        name="middle_stage_head",
    )
    middle_stage.visual(
        mesh_from_cadquery(
            _middle_stage_stop_collar_shape(), "middle_stage_stop_collar_mesh"
        ),
        material="service_black",
        name="middle_stage_stop_collar",
    )
    middle_stage.inertial = Inertial.from_geometry(
        Box((MIDDLE_OUTER, MIDDLE_OUTER, MIDDLE_TOTAL)),
        mass=7.2,
        origin=Origin(xyz=(0.0, 0.0, (MIDDLE_TOTAL / 2.0) - MIDDLE_INSERT)),
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        mesh_from_cadquery(_upper_stage_shell_shape(), "upper_stage_shell_mesh"),
        material="anodized_light",
        name="upper_stage_shell",
    )
    upper_stage.visual(
        mesh_from_cadquery(_upper_stage_mount_shape(), "upper_stage_mount_mesh"),
        material="service_black",
        name="upper_stage_mount_plate",
    )
    upper_stage.visual(
        mesh_from_cadquery(_upper_stage_stop_collar_shape(), "upper_stage_stop_collar_mesh"),
        material="service_black",
        name="upper_stage_stop_collar",
    )
    upper_stage.inertial = Inertial.from_geometry(
        Box((UPPER_MOUNT_PLATE, UPPER_MOUNT_PLATE, UPPER_TOTAL + UPPER_MOUNT_THICKNESS)),
        mass=4.4,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                ((UPPER_TOTAL + UPPER_MOUNT_THICKNESS) / 2.0) - UPPER_INSERT,
            )
        ),
    )

    pan_cartridge = model.part("pan_cartridge")
    pan_cartridge.visual(
        mesh_from_cadquery(_cartridge_base_shape(), "cartridge_base_mesh"),
        material="machined_silver",
        name="cartridge_base",
    )
    pan_cartridge.visual(
        mesh_from_cadquery(_cartridge_top_plate_shape(), "cartridge_top_plate_mesh"),
        material="machined_silver",
        name="cartridge_top_plate",
    )
    pan_cartridge.visual(
        mesh_from_cadquery(_cartridge_cap_shape(), "cartridge_cap_mesh"),
        material="service_black",
        name="cartridge_cap",
    )
    pan_cartridge.visual(
        mesh_from_cadquery(_cartridge_side_pod_shape(), "cartridge_side_pod_mesh"),
        material="service_black",
        name="cartridge_side_pod",
    )
    pan_cartridge.inertial = Inertial.from_geometry(
        Box(
            (
                CARTRIDGE_TOP_LENGTH + CARTRIDGE_POD_LENGTH * 0.35,
                CARTRIDGE_TOP_WIDTH,
                CARTRIDGE_BASE_HEIGHT + CARTRIDGE_TOP_THICKNESS + CARTRIDGE_CAP_HEIGHT,
            )
        ),
        mass=3.0,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (
                    CARTRIDGE_BASE_HEIGHT
                    + CARTRIDGE_TOP_THICKNESS
                    + CARTRIDGE_CAP_HEIGHT
                )
                / 2.0,
            )
        ),
    )

    model.articulation(
        "lower_to_middle_lift",
        ArticulationType.PRISMATIC,
        parent=lower_sleeve,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_THICKNESS + LOWER_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MIDDLE_TRAVEL,
            effort=160.0,
            velocity=0.24,
        ),
    )
    model.articulation(
        "middle_to_upper_lift",
        ArticulationType.PRISMATIC,
        parent=middle_stage,
        child=upper_stage,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_TOTAL - MIDDLE_INSERT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=UPPER_TRAVEL,
            effort=110.0,
            velocity=0.22,
        ),
    )
    model.articulation(
        "upper_to_cartridge_yaw",
        ArticulationType.REVOLUTE,
        parent=upper_stage,
        child=pan_cartridge,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (UPPER_TOTAL - UPPER_INSERT) + UPPER_MOUNT_THICKNESS - 0.004,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-2.5,
            upper=2.5,
            effort=28.0,
            velocity=1.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_sleeve = object_model.get_part("lower_sleeve")
    middle_stage = object_model.get_part("middle_stage")
    upper_stage = object_model.get_part("upper_stage")
    pan_cartridge = object_model.get_part("pan_cartridge")

    lower_to_middle = object_model.get_articulation("lower_to_middle_lift")
    middle_to_upper = object_model.get_articulation("middle_to_upper_lift")
    upper_to_cartridge = object_model.get_articulation("upper_to_cartridge_yaw")

    ctx.expect_within(
        middle_stage,
        lower_sleeve,
        axes="xy",
        inner_elem="middle_stage_shell",
        outer_elem="lower_sleeve_shell",
        margin=0.0,
        name="middle stage stays centered in the fixed lower sleeve",
    )
    ctx.expect_overlap(
        middle_stage,
        lower_sleeve,
        axes="z",
        elem_a="middle_stage_shell",
        elem_b="lower_sleeve_shell",
        min_overlap=0.30,
        name="middle stage retains deep insertion when collapsed",
    )
    ctx.expect_contact(
        middle_stage,
        lower_sleeve,
        elem_a="middle_stage_stop_collar",
        elem_b="lower_sleeve_guide",
        contact_tol=5e-4,
        name="middle stage is supported by the lower guide collar",
    )

    ctx.expect_within(
        upper_stage,
        middle_stage,
        axes="xy",
        inner_elem="upper_stage_shell",
        outer_elem="middle_stage_shell",
        margin=0.0,
        name="upper stage stays centered in the middle stage",
    )
    ctx.expect_overlap(
        upper_stage,
        middle_stage,
        axes="z",
        elem_a="upper_stage_shell",
        elem_b="middle_stage_shell",
        min_overlap=0.24,
        name="upper stage retains insertion when collapsed",
    )
    ctx.expect_contact(
        upper_stage,
        middle_stage,
        elem_a="upper_stage_stop_collar",
        elem_b="middle_stage_head",
        contact_tol=5e-4,
        name="upper stage is supported by the middle stage head",
    )

    middle_rest = ctx.part_world_position(middle_stage)
    with ctx.pose({lower_to_middle: MIDDLE_TRAVEL}):
        ctx.expect_within(
            middle_stage,
            lower_sleeve,
            axes="xy",
            inner_elem="middle_stage_shell",
            outer_elem="lower_sleeve_shell",
            margin=0.0,
            name="middle stage stays centered at full first-stage lift",
        )
        ctx.expect_overlap(
            middle_stage,
            lower_sleeve,
            axes="z",
            elem_a="middle_stage_shell",
            elem_b="lower_sleeve_shell",
            min_overlap=0.10,
            name="middle stage keeps retained insertion at full first-stage lift",
        )
        middle_extended = ctx.part_world_position(middle_stage)

    upper_rest = ctx.part_world_position(upper_stage)
    with ctx.pose({lower_to_middle: MIDDLE_TRAVEL, middle_to_upper: UPPER_TRAVEL}):
        ctx.expect_within(
            upper_stage,
            middle_stage,
            axes="xy",
            inner_elem="upper_stage_shell",
            outer_elem="middle_stage_shell",
            margin=0.0,
            name="upper stage stays centered at full two-stage lift",
        )
        ctx.expect_overlap(
            upper_stage,
            middle_stage,
            axes="z",
            elem_a="upper_stage_shell",
            elem_b="middle_stage_shell",
            min_overlap=0.10,
            name="upper stage keeps retained insertion at full two-stage lift",
        )
        upper_extended = ctx.part_world_position(upper_stage)

    ctx.check(
        "middle stage extends upward",
        middle_rest is not None
        and middle_extended is not None
        and middle_extended[2] > middle_rest[2] + 0.15,
        details=f"rest={middle_rest}, extended={middle_extended}",
    )
    ctx.check(
        "upper stage extends upward",
        upper_rest is not None
        and upper_extended is not None
        and upper_extended[2] > upper_rest[2] + 0.10,
        details=f"rest={upper_rest}, extended={upper_extended}",
    )

    ctx.expect_origin_distance(
        upper_stage,
        pan_cartridge,
        axes="xy",
        min_dist=0.0,
        max_dist=1e-5,
        name="cartridge stays on the mast centerline",
    )
    ctx.expect_contact(
        pan_cartridge,
        upper_stage,
        elem_a="cartridge_base",
        elem_b="upper_stage_mount_plate",
        contact_tol=5e-4,
        name="cartridge seats on the upper mast mount plate",
    )

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((low + high) / 2.0 for low, high in zip(lo, hi))

    cartridge_rest = ctx.part_world_position(pan_cartridge)
    pod_rest = _center_from_aabb(
        ctx.part_element_world_aabb(pan_cartridge, elem="cartridge_side_pod")
    )
    with ctx.pose({upper_to_cartridge: 1.0}):
        cartridge_yawed = ctx.part_world_position(pan_cartridge)
        pod_yawed = _center_from_aabb(
            ctx.part_element_world_aabb(pan_cartridge, elem="cartridge_side_pod")
        )

    pod_moved = False
    if pod_rest is not None and pod_yawed is not None:
        dx = pod_yawed[0] - pod_rest[0]
        dy = pod_yawed[1] - pod_rest[1]
        pod_moved = (dx * dx + dy * dy) ** 0.5 > 0.04 and abs(
            pod_yawed[2] - pod_rest[2]
        ) < 0.01

    ctx.check(
        "cartridge yaws about the shared centerline",
        cartridge_rest is not None
        and cartridge_yawed is not None
        and abs(cartridge_yawed[0] - cartridge_rest[0]) < 1e-5
        and abs(cartridge_yawed[1] - cartridge_rest[1]) < 1e-5
        and abs(cartridge_yawed[2] - cartridge_rest[2]) < 1e-5
        and pod_moved,
        details=(
            f"cartridge_rest={cartridge_rest}, cartridge_yawed={cartridge_yawed}, "
            f"pod_rest={pod_rest}, pod_yawed={pod_yawed}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
