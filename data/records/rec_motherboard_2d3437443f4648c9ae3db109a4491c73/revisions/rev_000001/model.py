from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BOARD_SIZE = (0.305, 0.244, 0.0016)
SOCKET_CENTER = (0.0, 0.018, BOARD_SIZE[2])
DIMM_SLOT_XS = (0.102, 0.115)
DIMM_SLOT_Y = 0.010
DIMM_SLOT_LENGTH = 0.134
DIMM_SLOT_WIDTH = 0.008
DIMM_SLOT_HEIGHT = 0.010
PIVOT_BLOCK_OUTER_Y = 0.070
LATCH_BARREL_RADIUS = 0.0018


def _build_load_plate_frame():
    outer_profile = [
        (0.001, -0.026),
        (0.046, -0.026),
        (0.046, -0.012),
        (0.053, -0.006),
        (0.053, 0.006),
        (0.046, 0.012),
        (0.046, 0.026),
        (0.001, 0.026),
    ]
    inner_profile = [
        (0.010, -0.018),
        (0.041, -0.018),
        (0.041, 0.018),
        (0.010, 0.018),
    ]

    frame = ExtrudeWithHolesGeometry(
        outer_profile,
        [inner_profile],
        0.0012,
        center=True,
    )
    bridge = BoxGeometry((0.0035, 0.040, 0.0012)).translate(0.00175, 0.0, 0.0)
    frame.merge(bridge)
    return frame


def _build_latch_body(inward_sign: float):
    body = BoxGeometry((0.003, 0.0026, 0.0148)).translate(0.0, inward_sign * 0.0013, 0.0074)
    arm = BoxGeometry((0.003, 0.0105, 0.0030)).translate(0.0, inward_sign * 0.0054, 0.0157)
    finger = BoxGeometry((0.003, 0.0075, 0.0060)).translate(0.0, inward_sign * 0.0097, 0.0194)
    body.merge(arm)
    body.merge(finger)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_motherboard")

    pcb_green = model.material("pcb_green", rgba=(0.10, 0.25, 0.14, 1.0))
    socket_black = model.material("socket_black", rgba=(0.09, 0.09, 0.10, 1.0))
    slot_black = model.material("slot_black", rgba=(0.13, 0.13, 0.14, 1.0))
    matte_gray = model.material("matte_gray", rgba=(0.42, 0.44, 0.46, 1.0))
    heatsink_gray = model.material("heatsink_gray", rgba=(0.55, 0.58, 0.60, 1.0))
    metal = model.material("metal", rgba=(0.77, 0.79, 0.81, 1.0))
    gold = model.material("gold", rgba=(0.82, 0.68, 0.27, 1.0))
    battery = model.material("battery", rgba=(0.72, 0.74, 0.77, 1.0))

    board = model.part("motherboard")
    board.visual(
        Box(BOARD_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BOARD_SIZE[2] / 2.0)),
        material=pcb_green,
        name="pcb",
    )
    board.visual(
        Box((0.016, 0.076, 0.040)),
        origin=Origin(xyz=(-0.1445, 0.070, 0.0216)),
        material=matte_gray,
        name="rear_io_stack",
    )
    board.visual(
        Box((0.070, 0.018, 0.016)),
        origin=Origin(xyz=(0.002, 0.066, 0.0096)),
        material=heatsink_gray,
        name="vrm_heatsink_top",
    )
    board.visual(
        Box((0.018, 0.062, 0.016)),
        origin=Origin(xyz=(-0.056, 0.022, 0.0096)),
        material=heatsink_gray,
        name="vrm_heatsink_side",
    )
    board.visual(
        Box((0.040, 0.040, 0.016)),
        origin=Origin(xyz=(0.020, -0.060, 0.0096)),
        material=heatsink_gray,
        name="chipset_heatsink",
    )
    board.visual(
        Box((0.092, 0.012, 0.013)),
        origin=Origin(xyz=(-0.006, -0.094, 0.0081)),
        material=slot_black,
        name="pcie_slot_primary",
    )
    board.visual(
        Box((0.058, 0.012, 0.013)),
        origin=Origin(xyz=(-0.084, -0.066, 0.0081)),
        material=slot_black,
        name="pcie_slot_secondary",
    )
    board.visual(
        Box((0.076, 0.022, 0.004)),
        origin=Origin(xyz=(0.028, -0.028, 0.0036)),
        material=matte_gray,
        name="m2_cover",
    )
    board.visual(
        Box((0.008, 0.054, 0.016)),
        origin=Origin(xyz=(0.145, 0.048, 0.0096)),
        material=matte_gray,
        name="power_header",
    )
    board.visual(
        Cylinder(radius=0.012, length=0.0032),
        origin=Origin(xyz=(0.108, -0.086, 0.0032)),
        material=battery,
        name="cmos_battery",
    )

    socket = model.part("cpu_socket")
    socket.visual(
        Box((0.060, 0.060, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=socket_black,
        name="socket_body",
    )
    socket.visual(
        Box((0.038, 0.038, 0.0008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0064)),
        material=gold,
        name="contact_field",
    )
    socket.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                [(-0.027, -0.027), (0.027, -0.027), (0.027, 0.027), (-0.027, 0.027)],
                [[(-0.019, -0.019), (0.019, -0.019), (0.019, 0.019), (-0.019, 0.019)]],
                0.0012,
                center=True,
            ),
            "socket_load_frame",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0066)),
        material=metal,
        name="load_frame",
    )
    socket.visual(
        Box((0.006, 0.040, 0.008)),
        origin=Origin(xyz=(-0.033, 0.0, 0.004)),
        material=metal,
        name="plate_hinge_support",
    )
    socket.visual(
        Box((0.004, 0.012, 0.008)),
        origin=Origin(xyz=(0.031, 0.020, 0.004)),
        material=metal,
        name="retention_post",
    )
    model.articulation(
        "board_to_socket",
        ArticulationType.FIXED,
        parent=board,
        child=socket,
        origin=Origin(xyz=SOCKET_CENTER),
    )

    load_plate = model.part("cpu_load_plate")
    load_plate.visual(
        mesh_from_geometry(_build_load_plate_frame(), "cpu_load_plate_frame"),
        material=metal,
        name="plate_frame",
    )
    load_plate.visual(
        Cylinder(radius=0.0015, length=0.040),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hinge_barrel",
    )
    load_plate.visual(
        Box((0.003, 0.036, 0.0024)),
        origin=Origin(xyz=(0.0, 0.0, -0.0012)),
        material=metal,
        name="hinge_knuckle",
    )
    model.articulation(
        "socket_to_load_plate",
        ArticulationType.REVOLUTE,
        parent=socket,
        child=load_plate,
        origin=Origin(xyz=(-0.0285, 0.0, 0.0084)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.75,
        ),
    )

    for index, slot_x in enumerate(DIMM_SLOT_XS, start=1):
        slot = model.part(f"dimm_slot_{index}")
        slot.visual(
            Box((DIMM_SLOT_WIDTH, DIMM_SLOT_LENGTH, DIMM_SLOT_HEIGHT)),
            origin=Origin(xyz=(0.0, 0.0, DIMM_SLOT_HEIGHT / 2.0)),
            material=slot_black,
            name="slot_body",
        )
        slot.visual(
            Box((0.0032, 0.050, 0.004)),
            origin=Origin(xyz=(0.0, -0.037, 0.012)),
            material=matte_gray,
            name="contact_ridge_lower",
        )
        slot.visual(
            Box((0.0032, 0.050, 0.004)),
            origin=Origin(xyz=(0.0, 0.037, 0.012)),
            material=matte_gray,
            name="contact_ridge_upper",
        )
        slot.visual(
            Box((0.006, 0.0032, 0.009)),
            origin=Origin(xyz=(0.0, -0.0684, 0.0045)),
            material=matte_gray,
            name="lower_pivot_block",
        )
        slot.visual(
            Box((0.006, 0.0032, 0.009)),
            origin=Origin(xyz=(0.0, 0.0684, 0.0045)),
            material=matte_gray,
            name="upper_pivot_block",
        )
        model.articulation(
            f"board_to_dimm_slot_{index}",
            ArticulationType.FIXED,
            parent=board,
            child=slot,
            origin=Origin(xyz=(slot_x, DIMM_SLOT_Y, BOARD_SIZE[2])),
        )

        for side_name, inward_sign, pivot_sign, axis_sign in (
            ("lower", 1.0, -1.0, 1.0),
            ("upper", -1.0, 1.0, -1.0),
        ):
            latch = model.part(f"dimm_slot_{index}_{side_name}_latch")
            latch.visual(
                mesh_from_geometry(
                    _build_latch_body(inward_sign),
                    f"dimm_slot_{index}_{side_name}_latch_body",
                ),
                material=matte_gray,
                name="latch_body",
            )
            latch.visual(
                Cylinder(radius=LATCH_BARREL_RADIUS, length=0.005),
                origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
                material=matte_gray,
                name="pivot_barrel",
            )
            model.articulation(
                f"dimm_slot_{index}_{side_name}_latch_hinge",
                ArticulationType.REVOLUTE,
                parent=slot,
                child=latch,
                origin=Origin(
                    xyz=(
                        0.0,
                        pivot_sign * (PIVOT_BLOCK_OUTER_Y + LATCH_BARREL_RADIUS),
                        0.0045,
                    )
                ),
                axis=(axis_sign, 0.0, 0.0),
                motion_limits=MotionLimits(
                    effort=1.0,
                    velocity=4.0,
                    lower=0.0,
                    upper=1.05,
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

    board = object_model.get_part("motherboard")
    socket = object_model.get_part("cpu_socket")
    load_plate = object_model.get_part("cpu_load_plate")
    load_plate_hinge = object_model.get_articulation("socket_to_load_plate")

    ctx.expect_gap(
        socket,
        board,
        axis="z",
        positive_elem="socket_body",
        negative_elem="pcb",
        max_gap=0.0002,
        max_penetration=1e-6,
        name="cpu socket sits flush on the motherboard PCB",
    )

    with ctx.pose({load_plate_hinge: 0.0}):
        ctx.expect_gap(
            load_plate,
            socket,
            axis="x",
            positive_elem="hinge_knuckle",
            negative_elem="plate_hinge_support",
            max_gap=0.0002,
            max_penetration=1e-6,
            name="load plate hinge barrel seats on the socket hinge support",
        )
        ctx.expect_overlap(
            load_plate,
            socket,
            axes="yz",
            elem_a="hinge_knuckle",
            elem_b="plate_hinge_support",
            min_overlap=0.0020,
            name="load plate hinge barrel aligns with the hinge support block",
        )
        ctx.expect_overlap(
            load_plate,
            socket,
            axes="xy",
            elem_a="plate_frame",
            elem_b="load_frame",
            min_overlap=0.040,
            name="closed load plate covers the socket opening",
        )
        ctx.expect_gap(
            load_plate,
            socket,
            axis="z",
            positive_elem="plate_frame",
            negative_elem="load_frame",
            min_gap=0.0005,
            max_gap=0.0020,
            name="closed load plate rests just above the metal load frame",
        )

    closed_plate_aabb = None
    open_plate_aabb = None
    with ctx.pose({load_plate_hinge: 0.0}):
        closed_plate_aabb = ctx.part_element_world_aabb(load_plate, elem="plate_frame")
    with ctx.pose({load_plate_hinge: 1.45}):
        open_plate_aabb = ctx.part_element_world_aabb(load_plate, elem="plate_frame")
    ctx.check(
        "load plate swings upward when opened",
        closed_plate_aabb is not None
        and open_plate_aabb is not None
        and open_plate_aabb[1][2] > closed_plate_aabb[1][2] + 0.040,
        details=f"closed={closed_plate_aabb}, open={open_plate_aabb}",
    )

    for index in (1, 2):
        slot = object_model.get_part(f"dimm_slot_{index}")
        ctx.expect_gap(
            slot,
            board,
            axis="z",
            positive_elem="slot_body",
            negative_elem="pcb",
            max_gap=0.0002,
            max_penetration=1e-6,
            name=f"dimm slot {index} mounts flush to the PCB",
        )

        for side_name, outward_direction in (("lower", "negative"), ("upper", "positive")):
            latch = object_model.get_part(f"dimm_slot_{index}_{side_name}_latch")
            latch_joint = object_model.get_articulation(f"dimm_slot_{index}_{side_name}_latch_hinge")
            pivot_block = f"{side_name}_pivot_block"

            with ctx.pose({latch_joint: 0.0}):
                if side_name == "upper":
                    ctx.expect_gap(
                        latch,
                        slot,
                        axis="y",
                        positive_elem="pivot_barrel",
                        negative_elem=pivot_block,
                        max_gap=0.0002,
                        max_penetration=1e-6,
                        name=f"dimm slot {index} upper latch sits on its pivot block",
                    )
                else:
                    ctx.expect_gap(
                        slot,
                        latch,
                        axis="y",
                        positive_elem=pivot_block,
                        negative_elem="pivot_barrel",
                        max_gap=0.0002,
                        max_penetration=1e-6,
                        name=f"dimm slot {index} lower latch sits on its pivot block",
                    )
                ctx.expect_overlap(
                    latch,
                    slot,
                    axes="xz",
                    elem_a="pivot_barrel",
                    elem_b=pivot_block,
                    min_overlap=0.0025,
                    name=f"dimm slot {index} {side_name} latch pivot stays aligned with its block",
                )

            closed_latch_aabb = None
            open_latch_aabb = None
            with ctx.pose({latch_joint: 0.0}):
                closed_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_body")
            with ctx.pose({latch_joint: 1.0}):
                open_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_body")

            if outward_direction == "positive":
                ok = (
                    closed_latch_aabb is not None
                    and open_latch_aabb is not None
                    and open_latch_aabb[1][1] > closed_latch_aabb[1][1] + 0.006
                )
            else:
                ok = (
                    closed_latch_aabb is not None
                    and open_latch_aabb is not None
                    and open_latch_aabb[0][1] < closed_latch_aabb[0][1] - 0.006
                )

            ctx.check(
                f"dimm slot {index} {side_name} latch rotates outward",
                ok,
                details=f"closed={closed_latch_aabb}, open={open_latch_aabb}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
