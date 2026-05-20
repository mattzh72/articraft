from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _dashboard_y(x: float, z: float) -> float:
    """Softly crowned dashboard surface: proud at center, falling back at edges."""
    x_term = (x / 0.56) ** 2
    z_term = ((z - 0.02) / 0.34) ** 2
    return 0.030 - 0.034 * x_term - 0.010 * z_term


def _curved_fascia_mesh() -> MeshGeometry:
    """One connected thick dashboard panel with a glove-box opening."""
    width = 1.12
    height = 0.52
    thickness = 0.038
    opening_w = 0.78
    opening_z_min = -0.165
    opening_z_max = 0.165
    nx = 40
    nz = 24

    base_xs = [(-width / 2.0) + width * i / nx for i in range(nx + 1)]
    base_zs = [(-height / 2.0) + height * i / nz for i in range(nz + 1)]
    xs = sorted(set(round(v, 6) for v in base_xs + [-opening_w / 2.0, opening_w / 2.0]))
    zs = sorted(set(round(v, 6) for v in base_zs + [opening_z_min, opening_z_max]))

    def in_opening(ix: int, iz: int) -> bool:
        cx = (xs[ix] + xs[ix + 1]) * 0.5
        cz = (zs[iz] + zs[iz + 1]) * 0.5
        return -opening_w / 2.0 < cx < opening_w / 2.0 and opening_z_min < cz < opening_z_max

    kept = {
        (ix, iz)
        for ix in range(len(xs) - 1)
        for iz in range(len(zs) - 1)
        if not in_opening(ix, iz)
    }

    mesh = MeshGeometry()
    front: dict[tuple[int, int], int] = {}
    back: dict[tuple[int, int], int] = {}

    def vertex(ix: int, iz: int, is_front: bool) -> int:
        store = front if is_front else back
        key = (ix, iz)
        if key not in store:
            x = xs[ix]
            z = zs[iz]
            y = _dashboard_y(x, z)
            if not is_front:
                y -= thickness
            store[key] = mesh.add_vertex(x, y, z)
        return store[key]

    for ix, iz in kept:
        f00 = vertex(ix, iz, True)
        f10 = vertex(ix + 1, iz, True)
        f11 = vertex(ix + 1, iz + 1, True)
        f01 = vertex(ix, iz + 1, True)
        b00 = vertex(ix, iz, False)
        b10 = vertex(ix + 1, iz, False)
        b11 = vertex(ix + 1, iz + 1, False)
        b01 = vertex(ix, iz + 1, False)

        _add_quad(mesh, f00, f10, f11, f01)
        _add_quad(mesh, b10, b00, b01, b11)

        for dx, dz, edge in (
            (-1, 0, "left"),
            (1, 0, "right"),
            (0, -1, "bottom"),
            (0, 1, "top"),
        ):
            if (ix + dx, iz + dz) in kept:
                continue
            if edge == "left":
                _add_quad(mesh, vertex(ix, iz, True), vertex(ix, iz + 1, True), vertex(ix, iz + 1, False), vertex(ix, iz, False))
            elif edge == "right":
                _add_quad(mesh, vertex(ix + 1, iz + 1, True), vertex(ix + 1, iz, True), vertex(ix + 1, iz, False), vertex(ix + 1, iz + 1, False))
            elif edge == "bottom":
                _add_quad(mesh, vertex(ix + 1, iz, True), vertex(ix, iz, True), vertex(ix, iz, False), vertex(ix + 1, iz, False))
            else:
                _add_quad(mesh, vertex(ix, iz + 1, True), vertex(ix + 1, iz + 1, True), vertex(ix + 1, iz + 1, False), vertex(ix, iz + 1, False))

    return mesh


def _curved_tray_floor_mesh() -> MeshGeometry:
    """A gently crowned, thick drop-door tray floor in the door frame."""
    width = 0.74
    length = 0.36
    thickness = 0.026
    nx = 28
    ny = 18
    mesh = MeshGeometry()
    top: dict[tuple[int, int], int] = {}
    bottom: dict[tuple[int, int], int] = {}

    def z_top(x: float, y: float) -> float:
        # A slight crown in the center and a raised front edge, like a molded door.
        return 0.012 + 0.025 * (y / length) - 0.010 * (x / (width / 2.0)) ** 2

    def vertex(ix: int, iy: int, is_top: bool) -> int:
        store = top if is_top else bottom
        key = (ix, iy)
        if key not in store:
            x = -width / 2.0 + width * ix / nx
            y = length * iy / ny
            z = z_top(x, y)
            if not is_top:
                z -= thickness
            store[key] = mesh.add_vertex(x, y, z)
        return store[key]

    for ix in range(nx):
        for iy in range(ny):
            t00 = vertex(ix, iy, True)
            t10 = vertex(ix + 1, iy, True)
            t11 = vertex(ix + 1, iy + 1, True)
            t01 = vertex(ix, iy + 1, True)
            b00 = vertex(ix, iy, False)
            b10 = vertex(ix + 1, iy, False)
            b11 = vertex(ix + 1, iy + 1, False)
            b01 = vertex(ix, iy + 1, False)
            _add_quad(mesh, t00, t10, t11, t01)
            _add_quad(mesh, b10, b00, b01, b11)

            if ix == 0:
                _add_quad(mesh, vertex(ix, iy, True), vertex(ix, iy + 1, True), vertex(ix, iy + 1, False), vertex(ix, iy, False))
            if ix == nx - 1:
                _add_quad(mesh, vertex(ix + 1, iy + 1, True), vertex(ix + 1, iy, True), vertex(ix + 1, iy, False), vertex(ix + 1, iy + 1, False))
            if iy == 0:
                _add_quad(mesh, vertex(ix + 1, iy, True), vertex(ix, iy, True), vertex(ix, iy, False), vertex(ix + 1, iy, False))
            if iy == ny - 1:
                _add_quad(mesh, vertex(ix, iy + 1, True), vertex(ix + 1, iy + 1, True), vertex(ix + 1, iy + 1, False), vertex(ix, iy + 1, False))
    return mesh


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curved_dashboard_glove_box")

    soft_dash = model.material("soft_dash", rgba=(0.42, 0.36, 0.29, 1.0))
    shadow_plastic = model.material("shadow_plastic", rgba=(0.045, 0.048, 0.050, 1.0))
    felt = model.material("dark_felt", rgba=(0.025, 0.030, 0.032, 1.0))
    hinge_steel = model.material("blackened_steel", rgba=(0.015, 0.014, 0.013, 1.0))
    satin_trim = model.material("satin_trim", rgba=(0.56, 0.53, 0.48, 1.0))

    fascia = model.part("fascia")
    fascia.visual(
        mesh_from_geometry(_curved_fascia_mesh(), "curved_fascia"),
        material=soft_dash,
        name="curved_fascia",
    )

    # Fixed inner storage tub: separate thin walls leave the front open and read
    # as a real bin rather than a solid block.
    fascia.visual(
        Box((0.72, 0.34, 0.030)),
        origin=Origin(xyz=(0.0, -0.185, -0.175)),
        material=shadow_plastic,
        name="tub_floor",
    )
    fascia.visual(
        Box((0.030, 0.34, 0.30)),
        origin=Origin(xyz=(-0.375, -0.185, -0.015)),
        material=shadow_plastic,
        name="tub_side_0",
    )
    fascia.visual(
        Box((0.030, 0.34, 0.30)),
        origin=Origin(xyz=(0.375, -0.185, -0.015)),
        material=shadow_plastic,
        name="tub_side_1",
    )
    fascia.visual(
        Box((0.72, 0.030, 0.30)),
        origin=Origin(xyz=(0.0, -0.355, -0.015)),
        material=shadow_plastic,
        name="tub_back",
    )
    fascia.visual(
        Box((0.72, 0.34, 0.026)),
        origin=Origin(xyz=(0.0, -0.185, 0.155)),
        material=shadow_plastic,
        name="tub_roof",
    )
    fascia.visual(
        Box((0.82, 0.024, 0.035)),
        origin=Origin(xyz=(0.0, -0.010, -0.178)),
        material=satin_trim,
        name="lower_lip",
    )
    fascia.visual(
        Box((0.82, 0.024, 0.026)),
        origin=Origin(xyz=(0.0, -0.010, 0.178)),
        material=satin_trim,
        name="upper_lip",
    )
    fascia.visual(
        Box((0.030, 0.024, 0.335)),
        origin=Origin(xyz=(-0.410, -0.010, 0.0)),
        material=satin_trim,
        name="side_lip_0",
    )
    fascia.visual(
        Box((0.030, 0.024, 0.335)),
        origin=Origin(xyz=(0.410, -0.010, 0.0)),
        material=satin_trim,
        name="side_lip_1",
    )

    # Support clevises near the lower corners, placed around (not through) the
    # hinge-arm shaft so the mechanical support is visible without broad overlap.
    for x, plate_name, cheek_inner_name, cheek_outer_name in (
        (-0.355, "support_plate_0", "clevis_cheek_0_0", "clevis_cheek_0_1"),
        (0.355, "support_plate_1", "clevis_cheek_1_0", "clevis_cheek_1_1"),
    ):
        fascia.visual(
            Box((0.070, 0.024, 0.070)),
            origin=Origin(xyz=(x, 0.020, -0.205)),
            material=hinge_steel,
            name=plate_name,
        )
        fascia.visual(
            Box((0.018, 0.105, 0.028)),
            origin=Origin(xyz=(x - 0.034, 0.065, -0.205)),
            material=hinge_steel,
            name=cheek_inner_name,
        )
        fascia.visual(
            Box((0.018, 0.105, 0.028)),
            origin=Origin(xyz=(x + 0.034, 0.065, -0.205)),
            material=hinge_steel,
            name=cheek_outer_name,
        )

    hinge_arms = model.part("hinge_arms")
    arm_dy = 0.180
    arm_dz = -0.045
    arm_len = math.hypot(arm_dy, arm_dz)
    arm_angle = math.atan2(arm_dz, arm_dy)
    for x, arm_name in ((-0.355, "short_arm_0"), (0.355, "short_arm_1")):
        hinge_arms.visual(
            Box((0.030, arm_len, 0.018)),
            origin=Origin(xyz=(x, arm_dy * 0.5, arm_dz * 0.5), rpy=(arm_angle, 0.0, 0.0)),
            material=hinge_steel,
            name=arm_name,
        )
    hinge_arms.visual(
        Cylinder(radius=0.012, length=0.77),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="support_shaft",
    )
    hinge_arms.visual(
        Cylinder(radius=0.010, length=0.74),
        origin=Origin(xyz=(0.0, arm_dy, arm_dz), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="door_pivot_shaft",
    )

    door = model.part("door")
    door.visual(
        mesh_from_geometry(_curved_tray_floor_mesh(), "tray_floor"),
        origin=Origin(xyz=(0.0, 0.024, 0.0)),
        material=soft_dash,
        name="tray_floor",
    )
    door.visual(
        Box((0.680, 0.275, 0.006)),
        origin=Origin(xyz=(0.0, 0.195, 0.034)),
        material=felt,
        name="tray_inlay",
    )
    door.visual(
        Box((0.026, 0.335, 0.060)),
        origin=Origin(xyz=(-0.380, 0.185, 0.035)),
        material=soft_dash,
        name="side_rail_0",
    )
    door.visual(
        Box((0.026, 0.335, 0.060)),
        origin=Origin(xyz=(0.380, 0.185, 0.035)),
        material=soft_dash,
        name="side_rail_1",
    )
    door.visual(
        Box((0.710, 0.030, 0.075)),
        origin=Origin(xyz=(0.0, 0.365, 0.048)),
        material=soft_dash,
        name="front_lip",
    )
    door.visual(
        Cylinder(radius=0.017, length=0.760),
        origin=Origin(xyz=(0.0, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_dash,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.760, 0.032, 0.030)),
        origin=Origin(xyz=(0.0, 0.032, 0.020)),
        material=soft_dash,
        name="hinge_sill",
    )
    door.visual(
        Box((0.260, 0.012, 0.054)),
        origin=Origin(xyz=(0.0, 0.350, 0.079)),
        material=satin_trim,
        name="handle_recess",
    )
    door.visual(
        Box((0.052, 0.017, 0.062)),
        origin=Origin(xyz=(-0.117, 0.3835, 0.064)),
        material=satin_trim,
        name="handle_mount_0",
    )
    door.visual(
        Box((0.052, 0.017, 0.062)),
        origin=Origin(xyz=(0.117, 0.3835, 0.064)),
        material=satin_trim,
        name="handle_mount_1",
    )

    handle = model.part("pull_handle")
    handle.visual(
        Cylinder(radius=0.009, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="handle_pin",
    )
    handle.visual(
        Box((0.240, 0.020, 0.055)),
        origin=Origin(xyz=(0.0, 0.017, -0.032)),
        material=shadow_plastic,
        name="handle_paddle",
    )
    handle.visual(
        Box((0.045, 0.022, 0.072)),
        origin=Origin(xyz=(-0.117, 0.014, -0.024)),
        material=shadow_plastic,
        name="handle_lug_0",
    )
    handle.visual(
        Box((0.045, 0.022, 0.072)),
        origin=Origin(xyz=(0.117, 0.014, -0.024)),
        material=shadow_plastic,
        name="handle_lug_1",
    )

    model.articulation(
        "support_pivot",
        ArticulationType.REVOLUTE,
        parent=fascia,
        child=hinge_arms,
        origin=Origin(xyz=(0.0, 0.065, -0.205)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.35, upper=0.70),
    )
    model.articulation(
        "door_pivot",
        ArticulationType.REVOLUTE,
        parent=hinge_arms,
        child=door,
        origin=Origin(xyz=(0.0, arm_dy, arm_dz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.4, lower=-0.20, upper=1.35),
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(0.0, 0.389, 0.088)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=0.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fascia = object_model.get_part("fascia")
    hinge_arms = object_model.get_part("hinge_arms")
    door = object_model.get_part("door")
    handle = object_model.get_part("pull_handle")
    support_pivot = object_model.get_articulation("support_pivot")
    door_pivot = object_model.get_articulation("door_pivot")
    handle_pivot = object_model.get_articulation("handle_pivot")

    ctx.allow_overlap(
        door,
        hinge_arms,
        elem_a="hinge_barrel",
        elem_b="door_pivot_shaft",
        reason="The door barrel is intentionally modeled around the lower hinge-arm shaft as a captured pivot.",
    )
    for arm_name in ("short_arm_0", "short_arm_1"):
        ctx.allow_overlap(
            door,
            hinge_arms,
            elem_a="hinge_barrel",
            elem_b=arm_name,
            reason="The end of each short hinge arm is represented as a compact eye blended into the door barrel.",
        )
    for cheek_name in (
        "clevis_cheek_0_0",
        "clevis_cheek_0_1",
        "clevis_cheek_1_0",
        "clevis_cheek_1_1",
    ):
        ctx.allow_overlap(
            fascia,
            hinge_arms,
            elem_a=cheek_name,
            elem_b="support_shaft",
            reason="The support shaft is modeled through the simplified solid clevis cheek where a real cheek has a pivot hole.",
        )
    for mount_name in ("handle_mount_0", "handle_mount_1"):
        ctx.allow_overlap(
            door,
            handle,
            elem_a=mount_name,
            elem_b="handle_pin",
            reason="The handle pin intentionally passes through the molded door mount as a captured pivot.",
        )
    ctx.expect_within(
        hinge_arms,
        door,
        axes="x",
        inner_elem="door_pivot_shaft",
        outer_elem="hinge_barrel",
        margin=0.020,
        name="hinge shaft length is captured by door barrel",
    )
    ctx.expect_overlap(
        door,
        hinge_arms,
        axes="yz",
        elem_a="hinge_barrel",
        elem_b="door_pivot_shaft",
        min_overlap=0.014,
        name="hinge shaft sits concentrically in door barrel",
    )
    ctx.expect_overlap(
        door,
        hinge_arms,
        axes="yz",
        elem_a="hinge_barrel",
        elem_b="short_arm_0",
        min_overlap=0.010,
        name="hinge arm eye reaches door barrel",
    )
    ctx.expect_overlap(
        fascia,
        hinge_arms,
        axes="yz",
        elem_a="clevis_cheek_0_1",
        elem_b="support_shaft",
        min_overlap=0.010,
        name="support shaft passes through clevis cheek",
    )

    ctx.expect_overlap(
        door,
        fascia,
        axes="x",
        elem_a="tray_floor",
        elem_b="lower_lip",
        min_overlap=0.60,
        name="drop door spans the glove-box opening",
    )
    ctx.expect_gap(
        door,
        fascia,
        axis="y",
        positive_elem="tray_floor",
        negative_elem="curved_fascia",
        min_gap=0.03,
        name="dropped tray projects out from fascia",
    )
    ctx.expect_gap(
        handle,
        door,
        axis="y",
        positive_elem="handle_paddle",
        negative_elem="front_lip",
        min_gap=0.002,
        max_gap=0.060,
        name="pull handle sits proud of the door face",
    )
    ctx.expect_contact(
        handle,
        door,
        elem_a="handle_lug_0",
        elem_b="handle_mount_0",
        contact_tol=0.001,
        name="handle lug is carried by door mount",
    )
    ctx.expect_overlap(
        door,
        handle,
        axes="yz",
        elem_a="handle_mount_0",
        elem_b="handle_pin",
        min_overlap=0.010,
        name="handle pin passes through door mount",
    )

    rest_arm_origin = ctx.part_world_position(door)
    with ctx.pose({support_pivot: 0.55}):
        lifted_arm_origin = ctx.part_world_position(door)
    ctx.check(
        "support arms swing the lower door pivots",
        rest_arm_origin is not None
        and lifted_arm_origin is not None
        and lifted_arm_origin[2] > rest_arm_origin[2] + 0.05,
        details=f"rest={rest_arm_origin}, lifted={lifted_arm_origin}",
    )

    rest_lip = _aabb_center(ctx.part_element_world_aabb(door, elem="front_lip"))
    with ctx.pose({door_pivot: 1.05}):
        raised_lip = _aabb_center(ctx.part_element_world_aabb(door, elem="front_lip"))
    ctx.check(
        "door rotates upward from tray position",
        rest_lip is not None and raised_lip is not None and raised_lip[2] > rest_lip[2] + 0.22,
        details=f"rest={rest_lip}, raised={raised_lip}",
    )

    rest_handle = _aabb_center(ctx.part_element_world_aabb(handle, elem="handle_paddle"))
    with ctx.pose({handle_pivot: 0.25}):
        pulled_handle = _aabb_center(ctx.part_element_world_aabb(handle, elem="handle_paddle"))
    ctx.check(
        "pull handle has a small independent rotation",
        rest_handle is not None
        and pulled_handle is not None
        and abs(pulled_handle[1] - rest_handle[1]) > 0.006,
        details=f"rest={rest_handle}, pulled={pulled_handle}",
    )

    return ctx.report()


object_model = build_object_model()
